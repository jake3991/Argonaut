# Installation

```
rosdep install --from-paths src --ignore-src -r -y

# MAVProxy and QGroundControl will automatically be installed when calling
# rosrun bluerov_launch mavproxy.sh
# rosrun bluerov_launch qgc.sh
```

# Build workspace

```
$ mkdir -p WORKSPACE/src
$ cd WORKSPACE
$ catkin init
$ catkin config --merge-devel
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Releas
```

# Manual installation

## Dependencies

```
#sudo apt-get install python-pip
sudo pip install catkin_tools mavproxy pymavlink scipy

# install ROS dependencies
sudo apt-get install ros-DISTRO-joy ros-DISTRO-cv-bridge
```

## QGroundControl

Before installing QGroundControl for the first time:

```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav -y
```

Logout and login again to enable the change to user permissions.

To install QGroundControl for Ubuntu Linux 16.04 LTS or later, Download [QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage)

```
chmod +x ./QGroundControl.AppImage
sudo mv ./QGroundControl.AppImage /usr/local/bin/QGroundControl.AppImage
```
