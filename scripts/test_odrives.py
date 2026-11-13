#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = []
# ///
"""Quick test harness for ODrives on a SocketCAN bus (CANSimple protocol).

    uv run odrive_can_test.py scan                 # list nodes, state, errors, vbus (no motion)
    uv run odrive_can_test.py vel  --node 0 --vel 1.0 --duration 3
    uv run odrive_can_test.py vel  --node 0 1 --vel 0.5           # both at same speed
    uv run odrive_can_test.py vel  --node all --vel 0.5 -0.5      # one speed per node
    uv run odrive_can_test.py pos  --node 0 1 --delta 0.25
    uv run odrive_can_test.py idle                 # put every node into IDLE
    uv run odrive_can_test.py clear                # clear errors on every node

Velocities are in turns/s, positions in turns. Any motion command puts the
axis back into IDLE on exit, including Ctrl-C and errors.
Uses only the stdlib (Linux AF_CAN sockets), so there are no dependencies.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from dataclasses import dataclass, field

# --- CANSimple command IDs (arbitration id = node_id << 5 | cmd) -------------
HEARTBEAT = 0x01
ESTOP = 0x02
GET_ERROR = 0x03
SET_AXIS_STATE = 0x07
GET_ENCODER_ESTIMATES = 0x09
SET_CONTROLLER_MODE = 0x0B
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
SET_LIMITS = 0x0F
GET_BUS_VOLTAGE_CURRENT = 0x15
GET_TEMPERATURE = 0x17
CLEAR_ERRORS = 0x18

AXIS_STATE = {0: "UNDEFINED", 1: "IDLE", 2: "STARTUP_SEQUENCE", 3: "FULL_CALIBRATION",
              4: "MOTOR_CALIBRATION", 6: "ENCODER_INDEX_SEARCH",
              7: "ENCODER_OFFSET_CALIBRATION", 8: "CLOSED_LOOP_CONTROL",
              9: "LOCKIN_SPIN", 10: "ENCODER_DIR_FIND", 11: "HOMING",
              12: "ENCODER_HALL_POLARITY_CAL", 13: "ENCODER_HALL_PHASE_CAL",
              14: "ANTICOGGING_CAL"}
IDLE, CLOSED_LOOP = 1, 8
CONTROL_VELOCITY, CONTROL_POSITION = 2, 3
INPUT_VEL_RAMP, INPUT_TRAP_TRAJ = 2, 5

# ODrive Pro/S1 (firmware 0.6.x) ODriveError bits
ODRIVE_ERRORS = {
    0x1: "INITIALIZING", 0x2: "SYSTEM_LEVEL", 0x4: "TIMING_ERROR",
    0x8: "MISSING_ESTIMATE", 0x10: "BAD_CONFIG", 0x20: "DRV_FAULT",
    0x40: "MISSING_INPUT", 0x100: "DC_BUS_OVER_VOLTAGE",
    0x200: "DC_BUS_UNDER_VOLTAGE", 0x400: "DC_BUS_OVER_CURRENT",
    0x800: "DC_BUS_OVER_REGEN_CURRENT", 0x1000: "CURRENT_LIMIT_VIOLATION",
    0x2000: "MOTOR_OVER_TEMP", 0x4000: "INVERTER_OVER_TEMP",
    0x8000: "VELOCITY_LIMIT_VIOLATION", 0x10000: "POSITION_LIMIT_VIOLATION",
    0x1000000: "WATCHDOG_TIMER_EXPIRED", 0x2000000: "ESTOP_REQUESTED",
    0x4000000: "SPINOUT_DETECTED", 0x8000000: "BRAKE_RESISTOR_DISARMED",
    0x10000000: "THERMISTOR_DISCONNECTED", 0x40000000: "CALIBRATION_ERROR",
}

CAN_FRAME = struct.Struct("<IB3x8s")
CAN_RTR_FLAG = 0x40000000


def decode_errors(bits: int) -> str:
    if bits == 0:
        return "none"
    names = [n for b, n in ODRIVE_ERRORS.items() if bits & b]
    unknown = bits & ~sum(ODRIVE_ERRORS)
    if unknown:
        names.append(f"0x{unknown:x}")
    return "|".join(names)


@dataclass
class NodeStatus:
    node_id: int
    axis_error: int = 0
    axis_state: int = 0
    procedure_result: int = 0
    trajectory_done: bool = False
    pos: float | None = None
    vel: float | None = None
    vbus: float | None = None
    ibus: float | None = None
    fet_temp: float | None = None
    motor_temp: float | None = None
    last_heartbeat: float = field(default=0.0)

    def summary(self) -> str:
        def f(v, fmt):
            return "?" if v is None else format(v, fmt)
        return (f"node {self.node_id}: state={AXIS_STATE.get(self.axis_state, self.axis_state)} "
                f"errors={decode_errors(self.axis_error)} proc_result={self.procedure_result} "
                f"pos={f(self.pos, '+.3f')}t vel={f(self.vel, '+.3f')}t/s "
                f"vbus={f(self.vbus, '.2f')}V ibus={f(self.ibus, '+.2f')}A "
                f"fet={f(self.fet_temp, '.1f')}C motor={f(self.motor_temp, '.1f')}C")


class ODriveBus:
    def __init__(self, iface: str):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.02)
        self.nodes: dict[int, NodeStatus] = {}

    # -- low level ---------------------------------------------------------
    def send(self, node: int, cmd: int, data: bytes = b"", rtr: bool = False) -> None:
        can_id = (node << 5) | cmd
        if rtr:
            can_id |= CAN_RTR_FLAG
        self.sock.send(CAN_FRAME.pack(can_id, len(data), data.ljust(8, b"\0")))

    def poll(self, duration: float) -> None:
        """Read frames for `duration` seconds and update node status."""
        end = time.monotonic() + duration
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                return
            self.sock.settimeout(min(remaining, 0.05))
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = CAN_FRAME.unpack(frame)
            if can_id & (CAN_RTR_FLAG | 0x80000000):  # ignore RTR / extended
                continue
            self._handle(can_id >> 5, can_id & 0x1F, data[:dlc])

    def _handle(self, node: int, cmd: int, d: bytes) -> None:
        st = self.nodes.setdefault(node, NodeStatus(node))
        if cmd == HEARTBEAT and len(d) >= 7:
            st.axis_error, st.axis_state, st.procedure_result, traj = struct.unpack_from("<IBBB", d)
            st.trajectory_done = bool(traj & 1)
            st.last_heartbeat = time.monotonic()
        elif cmd == GET_ENCODER_ESTIMATES and len(d) == 8:
            st.pos, st.vel = struct.unpack("<ff", d)
        elif cmd == GET_BUS_VOLTAGE_CURRENT and len(d) == 8:
            st.vbus, st.ibus = struct.unpack("<ff", d)
        elif cmd == GET_TEMPERATURE and len(d) == 8:
            st.fet_temp, st.motor_temp = struct.unpack("<ff", d)

    # -- commands ------------------------------------------------------------
    def set_axis_state(self, node: int, state: int) -> None:
        self.send(node, SET_AXIS_STATE, struct.pack("<I", state))

    def set_controller_mode(self, node: int, control_mode: int, input_mode: int) -> None:
        self.send(node, SET_CONTROLLER_MODE, struct.pack("<II", control_mode, input_mode))

    def set_input_vel(self, node: int, vel: float, torque_ff: float = 0.0) -> None:
        self.send(node, SET_INPUT_VEL, struct.pack("<ff", vel, torque_ff))

    def set_input_pos(self, node: int, pos: float) -> None:
        self.send(node, SET_INPUT_POS, struct.pack("<fhh", pos, 0, 0))

    def set_limits(self, node: int, vel_limit: float, current_limit: float) -> None:
        self.send(node, SET_LIMITS, struct.pack("<ff", vel_limit, current_limit))

    def clear_errors(self, node: int) -> None:
        self.send(node, CLEAR_ERRORS, b"\0")

    def request(self, node: int, cmd: int) -> None:
        self.send(node, cmd, rtr=True)

    # -- helpers -------------------------------------------------------------
    def wait_for_state(self, node: int, state: int, timeout: float = 2.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll(0.05)
            st = self.nodes.get(node)
            if st and st.axis_state == state:
                return True
            if st and st.axis_error:
                return False
        return False

    def require_node(self, node: int) -> NodeStatus:
        self.poll(0.3)
        st = self.nodes.get(node)
        if st is None or not st.last_heartbeat:
            sys.exit(f"node {node} not found on bus (seen: {sorted(self.nodes) or 'none'})")
        return st


# --- subcommands -------------------------------------------------------------

def resolve_nodes(bus: ODriveBus, spec: list[str] | None) -> list[int]:
    """Turn `--node 0 1` / `--node all` into a list of node ids present on the bus."""
    bus.poll(0.3)
    seen = sorted(n for n, s in bus.nodes.items() if s.last_heartbeat)
    if not spec or spec == ["all"]:
        if not seen:
            sys.exit("no ODrive heartbeats seen on the bus")
        return seen
    nodes = [int(n) for n in spec]
    missing = [n for n in nodes if n not in seen]
    if missing:
        sys.exit(f"node(s) {missing} not found on bus (seen: {seen or 'none'})")
    return nodes


def per_node(values: list[float], nodes: list[int], name: str) -> dict[int, float]:
    """One value applies to every node; otherwise give exactly one per node."""
    if len(values) == 1:
        return {n: values[0] for n in nodes}
    if len(values) != len(nodes):
        sys.exit(f"--{name} needs 1 value or {len(nodes)} (one per node), got {len(values)}")
    return dict(zip(nodes, values))


def cmd_scan(bus: ODriveBus, args) -> None:
    bus.poll(0.5)
    for n in list(bus.nodes):
        bus.request(n, GET_BUS_VOLTAGE_CURRENT)
        bus.request(n, GET_TEMPERATURE)
    bus.poll(0.6)
    nodes = [s for s in bus.nodes.values() if s.last_heartbeat]
    if not nodes:
        print("no ODrive heartbeats seen on the bus")
        return
    for st in sorted(nodes, key=lambda s: s.node_id):
        print(st.summary())


def _enter_closed_loop(bus: ODriveBus, args, nodes: list[int],
                       control_mode: int, input_mode: int) -> None:
    errored = {n: bus.nodes[n].axis_error for n in nodes if bus.nodes[n].axis_error}
    if errored:
        sys.exit("active errors, run `clear` first: " +
                 ", ".join(f"node {n}: {decode_errors(e)}" for n, e in errored.items()))
    for n in nodes:
        bus.set_controller_mode(n, control_mode, input_mode)
        if args.vel_limit is not None:
            bus.set_limits(n, args.vel_limit, args.current_limit)
        bus.set_axis_state(n, CLOSED_LOOP)
    for n in nodes:
        if not bus.wait_for_state(n, CLOSED_LOOP):
            st = bus.nodes[n]
            raise RuntimeError(f"node {n} failed to enter CLOSED_LOOP_CONTROL: "
                               f"state={AXIS_STATE.get(st.axis_state)} "
                               f"errors={decode_errors(st.axis_error)} "
                               f"proc_result={st.procedure_result}")
    print(f"node(s) {nodes} in CLOSED_LOOP_CONTROL")


def _monitor(bus: ODriveBus, nodes: list[int], duration: float,
             done=lambda st: False) -> None:
    """Watch all nodes; abort if any drops out. Returns early once all are `done`."""
    end = time.monotonic() + duration
    next_print = 0.0
    while time.monotonic() < end:
        bus.poll(0.05)
        for n in nodes:
            st = bus.nodes[n]
            if st.axis_error or st.axis_state != CLOSED_LOOP:
                raise RuntimeError(f"node {n} dropped out: state={AXIS_STATE.get(st.axis_state)} "
                                   f"errors={decode_errors(st.axis_error)}")
        if time.monotonic() >= next_print:
            print("  " + "   ".join(f"[{n}] pos={bus.nodes[n].pos:+.3f}t vel={bus.nodes[n].vel:+.3f}t/s"
                                    for n in nodes))
            next_print = time.monotonic() + 0.25
        if all(done(bus.nodes[n]) for n in nodes):
            return


def cmd_vel(bus: ODriveBus, args) -> None:
    vels = per_node(args.vel, args.nodes, "vel")
    _enter_closed_loop(bus, args, args.nodes, CONTROL_VELOCITY, INPUT_VEL_RAMP)
    print("spinning " + ", ".join(f"node {n} @ {v:+.2f} t/s" for n, v in vels.items())
          + f" for {args.duration:.1f}s")
    for n, v in vels.items():
        bus.set_input_vel(n, v)
    _monitor(bus, args.nodes, args.duration)
    print("ramping down")
    for n in args.nodes:
        bus.set_input_vel(n, 0.0)
    _monitor(bus, args.nodes, 3.0, done=lambda st: st.vel is not None and abs(st.vel) < 0.05)


def cmd_pos(bus: ODriveBus, args) -> None:
    deltas = per_node(args.delta, args.nodes, "delta")
    bus.poll(0.1)
    if any(bus.nodes[n].pos is None for n in args.nodes):
        sys.exit("no encoder estimates received; cannot do a relative move")
    start = {n: bus.nodes[n].pos for n in args.nodes}
    _enter_closed_loop(bus, args, args.nodes, CONTROL_POSITION, INPUT_TRAP_TRAJ)
    for leg in ("out", "back"):
        targets = {n: start[n] + (deltas[n] if leg == "out" else 0.0) for n in args.nodes}
        print("moving " + ", ".join(f"node {n} -> {t:+.3f}t" for n, t in targets.items()))
        for n, t in targets.items():
            bus.set_input_pos(n, t)
        bus.poll(0.1)  # let trajectory_done clear
        _monitor(bus, args.nodes, args.timeout,
                 done=lambda s: s.trajectory_done and abs(s.pos - targets[s.node_id]) < 0.02)
        bus.poll(args.dwell)


def cmd_idle(bus: ODriveBus, args) -> None:
    for n in args.nodes:
        bus.set_axis_state(n, IDLE)
        print(f"node {n} -> IDLE")


def cmd_clear(bus: ODriveBus, args) -> None:
    for n in args.nodes:
        bus.clear_errors(n)
    bus.poll(0.3)
    for n in args.nodes:
        print(bus.nodes[n].summary())


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--iface", default="can0")
    p.add_argument("--vel-limit", type=float, default=None,
                   help="optionally send Set_Limits (turns/s) before moving")
    p.add_argument("--current-limit", type=float, default=10.0,
                   help="current limit (A) used with --vel-limit")
    sub = p.add_subparsers(dest="cmd", required=True)

    def add_node_arg(sp, default):
        sp.add_argument("--node", nargs="+", default=default, metavar="ID",
                        help="node id(s), or 'all' (default: %(default)s)")

    sub.add_parser("scan")

    v = sub.add_parser("vel")
    add_node_arg(v, ["0"])
    v.add_argument("--vel", type=float, nargs="+", default=[0.5],
                   help="turns/s; one value for all nodes or one per node")
    v.add_argument("--duration", type=float, default=3.0)

    ps = sub.add_parser("pos")
    add_node_arg(ps, ["0"])
    ps.add_argument("--delta", type=float, nargs="+", default=[0.25],
                    help="relative move in turns (then returns); one value or one per node")
    ps.add_argument("--timeout", type=float, default=5.0)
    ps.add_argument("--dwell", type=float, default=0.5)

    add_node_arg(sub.add_parser("idle"), ["all"])
    add_node_arg(sub.add_parser("clear"), ["all"])

    args = p.parse_args()
    bus = ODriveBus(args.iface)
    handler = {"scan": cmd_scan, "vel": cmd_vel, "pos": cmd_pos,
               "idle": cmd_idle, "clear": cmd_clear}[args.cmd]
    if args.cmd != "scan":
        args.nodes = resolve_nodes(bus, args.node)
    if args.cmd == "vel":  # validate before anything moves
        per_node(args.vel, args.nodes, "vel")
    elif args.cmd == "pos":
        per_node(args.delta, args.nodes, "delta")

    moving = args.cmd in ("vel", "pos")
    try:
        handler(bus, args)
    except KeyboardInterrupt:
        print("\ninterrupted")
    except RuntimeError as e:
        print(f"error: {e}")
    finally:
        if moving:
            for n in args.nodes:
                if args.cmd == "vel":
                    bus.set_input_vel(n, 0.0)
                bus.set_axis_state(n, IDLE)
            bus.poll(0.2)
            print("back to IDLE")
            for n in args.nodes:
                print("  " + bus.nodes[n].summary())


if __name__ == "__main__":
    main()
