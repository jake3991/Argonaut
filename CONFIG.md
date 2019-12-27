# Configuration

## BlueROV2

### USB devices 

Create serial rules for devices in `/etc/udev/rules.d/99-usb-serial.rules`, which is intended to set up a consistent device name. The attributes can be found by `udevadm info`, e.g.,  `udevadm info -a /dev/ttyUSB0 | grep 'idVendor' | head -n1`.

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0856", ATTRS{idProduct}=="ac11", ATTRS{serial}=="BB9O7S8F", SYMLINK+="rowe_dvl", MODE="0777"
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", ATTRS{serial}=="A703G09Z", SYMLINK+="rowe_dvl", MODE="0777"
```

### Remote serial ports

ssh to `pi@192.169.2.2` and add the following lines to `companion/.companion.rc` to enable virtual serial ports over TCP.

- DVL: 14660
- IMU: 14661

```
socat tcp-listen:14661,reuseaddr,fork,ignoreeof file:/dev/ttyAMA0,nonblock,waitlock=/var/run/ttyAMA0.lock,b115200,raw,echo=0 &
socat tcp-listen:14660,reuseaddr,fork file:/dev/rowe_dvl,nonblock,waitlock=/var/run/rowe_dvl.lock,b115200,raw,echo=0 &
```

### Disable camera

ssh to `pi@192.168.2.2` and comment the following line in `companion/.companion.rc`.

```
sudo -H -u pi screen -dm -S video $COMPANION_DIR/tools/streamer.py
```

## Topside

### IP addresses

- Computer: `192.168.2.1`
- BlueROV2 Rasberry PI: `192.168.2.2` user:password: pi:companion
- BlueROV2 Jetson Nano: `192.168.2.1` user:password: rov2:123
- Oculus M750d: `192.168.2.3`
- Oculus M120d: `192.168.2.4`


### QGroundControl

- In General, disable auto connect to UDP and add two UDP Comm Links on ports 14550 and 14551.
    - Use 14551 with ROS as the messages are directed to 14551 (see [mavproxy.sh](./bluerov_launch/scripts/mavproxy.sh)).
    - Use 14550 without ROS.

- Calibrate joystick.
  - [b0]/[b1]: arm/disarm
  - [b8]/[b3]: gain inc/dec
  - [a1]/[a0]/[a5]/[a2]: x/y/z/yaw
  - [a3]/[a4]: x/yaw cruise control

```
[b6] [b7]                     [b10]                  |
                                                     |
      [a1 + ]                  [b8]          [a4+|-] | [b11]
[a0 -][a2-|+][a0 +]        [b2]    [b9]              | [a5 u+]
      [a1 - ]                  [b3]          [a3+|-] | [b5]
                                                     |
[b0] [b1]                     [b4]                   |
```

### Set salinity

For pressure sensor, the salinity can be configured in the launch file. But currently for DVL, configure the salinity by running

```
rosrun rti_dvl minicom.sh

# Type
STOP<ENTER>
CWS 0 => fresh 35 => salt <ENTER>
START<ENTER>
```
