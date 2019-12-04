#!/usr/bin/env bash

# 14551 - qgc
# 14552 - depth
# 14553 - control
# 14554 - dashboard

if [ ! -x "$(command -v mavproxy.py)" ]; then
    echo "MAVProxy not found..."
    echo "Install MAVProxy..."
    sudo pip install MAVProxy
fi

# Otherwise mavlink can't receive any message!
timeout 1s bash -c "echo -ne '\n' | netcat -ul 14550"
mavproxy.py --master=udp:192.168.2.1:14550 --out=udp:192.168.2.1:14551 --out=udp:192.168.2.1:14552 --out=udp:192.168.2.1:14553 --out=udp:192.168.2.1:14554 --speech --logfile /tmp/mav.tlog