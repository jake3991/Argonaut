#!/usr/bin/env bash

if ! [ -x "$(command -v minicom)" ]; then
  echo "minicom not found"
  sudo apt-get install minicom
fi

socat pty,link=/tmp/rti_dvl,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14660 &
sleep 1
minicom -D /tmp/rti_dvl
killall socat
