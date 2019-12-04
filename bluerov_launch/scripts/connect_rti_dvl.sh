#!/usr/bin/env bash
socat pty,link=/tmp/rti_dvl,waitslave,raw,echo=0,ignoreeof tcp:192.168.2.2:14660