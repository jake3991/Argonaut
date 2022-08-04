#!/usr/bin/env python
import numpy as np
import cv2
from scipy.interpolate import interp1d

import rospy
import cv_bridge
from sonar_oculus.msg import OculusPingDeprecated
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server

REVERSE_Z = 1
global res, height, rows, width, cols, map_x, map_y, f_bearings
res, height, rows, width, cols = None, None, None, None, None
map_x, map_y = None, None
f_bearings = None

bridge = cv_bridge.CvBridge()

to_rad = lambda bearing: bearing * np.pi / 18000


def generate_map_xy(ping):
    _res = ping.range_resolution
    _height = ping.num_ranges * _res
    _rows = int(np.ceil(_height / _res))
    _width = np.sin(to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2
    _cols = int(np.ceil(_width / _res))

    global res, height, rows, width, cols, map_x, map_y, f_bearings
    if res == _res and height == _height and rows == _rows and width == _width and cols == _cols:
        return
    res, height, rows, width, cols = _res, _height, _rows, _width, _cols

    bearings = to_rad(np.asarray(ping.bearings, dtype=np.float32))
    f_bearings = interp1d(bearings, range(len(bearings)), kind='linear', fill_value='extrapolate')

    XX, YY = np.meshgrid(range(cols), range(rows))
    x = res * (rows - YY)
    y = res * (-cols / 2.0 + XX + 0.5)
    b = np.arctan2(y, x) * REVERSE_Z
    r = np.sqrt(np.square(x) + np.square(y))
    map_y = np.asarray(r / res, dtype=np.float32)
    map_x = np.asarray(f_bearings(b), dtype=np.float32)


def ping_callback(msg):
    raw = rospy.get_param('/sonar_oculus_node/Raw', False)
    cm = rospy.get_param('/sonar_oculus_node/Colormap', 1)
    if raw:
        img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding='passthrough')
        img = cv2.normalize(img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        img = cv2.applyColorMap(img, cm)
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg) 
    else:
        generate_map_xy(msg)

        img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding='passthrough')
        img = np.array(img, dtype=img.dtype, order='F')

        img.resize(rows, cols)
        img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

        # img_msg = bridge.cv2_to_imgmsg(img, encoding="mono8")

        img = cv2.normalize(img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        img = cv2.applyColorMap(img, cm)
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg) 


if __name__ == '__main__':
    rospy.init_node('oculus_viewer')
    img_pub = rospy.Publisher('/sonar_oculus_node/image', Image, queue_size=10)
    ping_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPingDeprecated, ping_callback, None, 10)

    rospy.spin()
