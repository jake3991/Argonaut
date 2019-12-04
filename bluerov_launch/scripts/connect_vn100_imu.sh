#!/usr/bin/env bash
socat pty,link=/tmp/vn100_imu,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14661