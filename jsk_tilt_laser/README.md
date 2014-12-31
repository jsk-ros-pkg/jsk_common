# Dynamixel users
Need to set udev to get permission to `/dev/ttyUSB0`

Please add `/etc/udev/rules.d/80-dynamixel.rules` file with following content:
```
SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", MODE="0666"
SUBSYSTEMS=="usb", KERNEL=="ttyUSB[0-9]*", MODE="0666"
```
