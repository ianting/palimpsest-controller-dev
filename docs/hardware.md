# Hardware Configuration

This file describes the hardware used in the Palimpsest Cable-driven Parallel Robot.

## Raspberry Pi

The controller runs on a rasberry pi that can be accessed via SSH over the Pi's USB cable.

OS: Ubuntu 24.04
Hostname: palimpsest
Username: user
Static IP: 10.42.0.2

```bash
ssh user@palimpsest.local # no password needed
```

Once a web interface exists for the robot, it could be accessed at [palimpsest.local](http://palimpsest.local).

## Motors

The project uses QDD brushless motors each with a planetary gear reduction.

## CAN Bus Communication

The Pi communicates with the ODrive brushless motor controllers over CAN bus, traveling through CAT6 cable.
