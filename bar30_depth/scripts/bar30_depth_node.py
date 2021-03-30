#!/usr/bin/env python
import sys
import socket
import rospy
from bar30_depth.msg import Depth

from bluerov_bridge import Bridge

FLUID_DENSITY = {'fresh': 9.97, 'salt': 10.29}

if __name__ == '__main__':
    rospy.init_node('bar30_depth_node')
    device = rospy.get_param('~device', 'udp:192.168.2.1:14552')

    if not rospy.has_param('/water'):
        rospy.set_param('/water', 'salt')
    water = rospy.get_param('/water', 'fresh')

    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device, write=False)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(
                    device))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    bridge.update()

    depth_pub = rospy.Publisher('/bar30/depth/raw', Depth, queue_size=10)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        msg = bridge.get_msg(type='SCALED_PRESSURE2')
        if msg is not None:
            d = Depth()
            d.header.stamp = rospy.Time.now()
            d.time = msg.time_boot_ms / 1000.0
            d.pressure_abs = msg.press_abs
            d.pressure_diff = msg.press_diff
            d.temperature = msg.temperature / 100.0
            # Assume pressure_diff is temperature compensated
            # https://github.com/bluerobotics/ardusub/blob/978cd64a1e3b0cb5ba1f3bcc995fcc39bea7e9ff/libraries/AP_Baro/AP_Baro_MS5611.cpp#L481
            # https://github.com/bluerobotics/ms5837-python/blob/c83bdc969ea1654a2e2759783546245709bd9914/ms5837.py#L146
            d.depth = d.pressure_diff / (FLUID_DENSITY[water] * 9.80665)
            depth_pub.publish(d)
        rate.sleep()
