#!/usr/bin/env bash

if [ ! -f "/usr/local/bin/QGroundControl.AppImage" ]; then
    echo "QGroundControl not found..."
    echo "Install QGroundControl..."

    sudo usermod -a -G dialout $USER
    sudo apt-get remove modemmanager -y
    sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav -y

    sudo wget https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage -P /usr/local/bin/
    sudo chmod +x /usr/local/bin/QGroundControl.AppImage
fi

/usr/local/bin/QGroundControl.AppImage &> /dev/null