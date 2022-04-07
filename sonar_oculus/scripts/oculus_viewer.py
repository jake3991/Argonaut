#!/usr/bin/env python
import numpy as np
import cv2
import sys
from scipy.interpolate import interp1d

import rospy
import cv_bridge
from sonar_oculus.msg import OculusPing
from sensor_msgs.msg import Image
from dynamic_reconfigure.server import Server

REVERSE_Z = 1
global res, height, rows, width, cols, map_x, map_y, f_bearings
res, height, rows, width, cols = None, None, None, None, None
map_x, map_y = None, None
f_bearings = None

bridge = cv_bridge.CvBridge()

to_rad = lambda bearing: bearing * np.pi / 18000

vis_lines = True

def generate_map_xy(ping):
    _res = ping.range_resolution
    _height = ping.num_ranges * _res
    _rows = ping.num_ranges
    _width = np.sin(
        to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2
    _cols = int(np.ceil(_width / _res))

    global res, height, rows, width, cols, map_x, map_y, f_bearings
    if res == _res and height == _height and rows == _rows and width == _width and cols == _cols:
        return
    res, height, rows, width, cols = _res, _height, _rows, _width, _cols

    bearings = to_rad(np.asarray(ping.bearings, dtype=np.float32))
    f_bearings = interp1d(
        bearings,
        range(len(bearings)),
        kind='linear',
        bounds_error=False,
        fill_value=-1,
        assume_sorted=True)

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

    #decode the compressed image
    img = np.fromstring(msg.ping.data,np.uint8)
    img = cv2.imdecode(img,cv2.IMREAD_COLOR)

    if raw:

        #img = bridge.imgmsg_to_cv2(msg.ping, desired_encoding='passthrough')
        img = cv2.normalize(
            img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        img = cv2.applyColorMap(img, cm)
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg)
    else:
        generate_map_xy(msg)

        img = np.array(img, dtype=img.dtype, order='F')

        if cols > img.shape[1]:
            img.resize(rows, cols)

        if vis_lines:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            cv2.line(img,(334,0),(334,1000),[0,255,0],5)
            cv2.line(img,(177,0),(177,1000),[0,255,0],5)


        img = cv2.remap(img, map_x, map_y, cv2.INTER_LINEAR)

        img = cv2.normalize(
            img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        img = cv2.applyColorMap(img, cm)
        img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg)


if __name__ == '__main__':

    #if no arguments are passed kill the node	
    if len(sys.argv) < 2:
        print("Please enter an argument for sonar model")
        print("for example use rosrun sonar_oculus oculus_viewer.py M1200d")
        rospy.signal_shutdown("Shutdown")

    #if arguments are a valid sonar model, set up the node
    elif (sys.argv[1] == 'M1200d') or (sys.argv[1] == 'M750d'):
        print("Succesfull startup, publishing " + sys.argv[1] + " Sonar")
        rospy.init_node('oculus_viewer_'+sys.argv[1])
        topic = rospy.get_param('~topic', '/sonar_oculus_node/'+sys.argv[1]+"/ping")
        ping_sub = rospy.Subscriber(topic, OculusPing,
                                ping_callback, None, 10)
        img_pub = rospy.Publisher(topic.rsplit('/', 1)[0] + '/image', Image, queue_size=10)
        rospy.spin()

    #otherwise its a typo, kill the node
    else:
        print("Please enter an argument for sonar model, you have entered a typo as your sonar model")
        print("for example use rosrun sonar_oculus oculus_viewer.py M1200d")
        rospy.signal_shutdown("Shutdown")
 

