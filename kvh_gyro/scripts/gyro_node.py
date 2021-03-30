#!/usr/bin/env python
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
from time import time
import struct
import numpy as np
import rospy
from kvh_gyro.msg import gyro

if __name__ == '__main__':

    #init the node
    rospy.init_node('gyro_node')

    #define the publisher
    pub = rospy.Publisher('/gyro', gyro, queue_size=1000)

    #define some parameters
    serialPort = "/dev/ttyUSB0" # RS422 converter port for IMU
    baudrate = 921600   #default baud rate, runs at 1000hz
    packetSepChar = '\xfe\x81\xff\x55'
    imuStatus = 'unknown'
    l = ['']   # List of queued packets
    p = ''     # Packet to parse

    # Initialize serial connection.
    try:
        ser = serial.Serial(serialPort, baudrate, timeout=0)
        rospy.loginfo("Starting Gyro")
    except serial.SerialException:
        rospy.loginfo("Failed to Start Gyro")
        exit(1)

    #while loop to get the data from the gyroscope
    while rospy.is_shutdown() == False:

        d = ser.readline()   # Raw data
        l = l[:-1] + (l[-1] + d).split(packetSepChar)   # Get packets. The last element in l may not necessarily be a whole packet.

        # If we have at least one whole packet
        if len(l) > 1:
            try:
                p = l[0]
                l = l[1:]   # Pop off packet that was just read.

                # Parse gyro data as big-endian floats.
                if len(p) == 32:
                    dx = struct.unpack('>f',   p[:4])[0] 
                    dy = struct.unpack('>f',  p[4:8])[0] 
                    dz = struct.unpack('>f', p[8:12])[0]
                    imuStatus = bin(ord(p[24]))

                    #publish the gyro data
                    msg = gyro()
                    msg.status = True
                    msg.header.stamp = rospy.Time.now()
                    msg.delta = [dx, dy, dz]
                    pub.publish(msg)

                else:
                    #publish an error message, somthing went wrong
                    msg = gyro()
                    msg.status = False
                    msg.header.stamp = rospy.Time.now()
                    pub.publish(msg)
            except:
                pass   # Sometimes we'll mangle the first packet. Ignore this.