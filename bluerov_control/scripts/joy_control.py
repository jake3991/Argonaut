#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

from bluerov_bridge import Bridge


cmd_vel_enabled = False
armed = False
depth_hold = False
x, y, z, yaw = 1500, 1500, 1500, 1500


def cmd_vel_sub(msg):
    if not cmd_vel_enabled:
        return

    global x, y, z, yaw
    vel_x, vel_y = msg.linear.x, msg.linear.y
    omega_z = msg.angular.z

    vel_x = max(-max_vel, min(max_vel, vel_x))
    vel_y = max(-max_vel, min(max_vel, vel_y))
    omega_z = max(-max_omega, min(max_omega, omega_z))

    x = 1500 + int(vel_to_cmd * vel_x)
    y = 1500 + int(vel_to_cmd * vel_y)
    z = 65535
    yaw = 1500 + int(omega_to_cmd * omega_z)


def joy_callback(msg):
    global x, y, z, yaw, armed, cmd_vel_enabled, depth_hold

    # Arm / disarm
    if msg.buttons[0]:
        armed = True
        rospy.loginfo('Arm the vehicle...')
    if msg.buttons[1]:
        armed = False
        rospy.loginfo('Disarm the vehicle...')

    if msg.buttons[8]:
        cmd_vel_enabled = True
        rospy.loginfo('Enable move_base control...')
    if msg.buttons[3]:
        cmd_vel_enabled = False
        rospy.loginfo('Disable move_base control...')

    # Depth hold / manual
    if msg.buttons[2]:
        depth_hold = True
        rospy.loginfo('Turn on depth hold mode...')
    if msg.buttons[9]:
        depth_hold = False
        rospy.loginfo('Turn on manual mode...')
    
    if cmd_vel_enabled:
        return

    # Movement
    x1 = 1500 + int(msg.axes[1] * translation_limit)
    y1 = 1500 + int(msg.axes[0] * translation_limit)
    z = 1500 + int(msg.axes[5] * translation_limit)
    yaw1 = 1500 - int(msg.axes[2] * rotation_limit)

    # Cruise control
    x2 = 1500 + int(msg.axes[3] * translation_limit)
    y2 = 1500
    yaw2 = 1500 - int(msg.axes[4] * rotation_limit)

    # Normal movement has higher priority
    x = x1 if x1 != 1500 else x2
    y = y1 if y1 != 1500 else y2
    yaw = yaw1 if yaw1 != 1500 else yaw2


if __name__ == '__main__':
    rospy.init_node('bluerov_cruise_control_node')

    translation_limit = rospy.get_param('~translation_limit', 100)
    rotation_limit = rospy.get_param('~rotation_limit', 80)
    max_vel = rospy.get_param('~max_vel', 0.2)
    max_omega = rospy.get_param('~max_omega', 0.15)
    vel_to_cmd = translation_limit / max_vel
    omega_to_cmd = rotation_limit / max_omega

    device = 'udp:192.168.2.1:14553'
    while not rospy.is_shutdown():
        try:
            bridge = Bridge(device)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(device)
            )
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)
    bridge.wait_conn()

    joy_sub = rospy.Subscriber('/joy', Joy, joy_callback, queue_size=10)
    cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, cmd_vel_sub)

    while not rospy.is_shutdown():
        bridge.set_mode('manual')
        bridge.arm_throttle(False)
        mode, arm = bridge.get_mode()
        if mode == 'MANUAL' and not arm:
            break
        rospy.sleep(0.5)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        bridge.update()

        if armed:
            bridge.arm_throttle(True)
        else:
            bridge.arm_throttle(False)

        if depth_hold:
            bridge.set_mode('alt_hold')
        else:
            bridge.set_mode('manual')

        bridge.set_cmd_vel(x, y, z, yaw)
        rate.sleep()

    while not rospy.is_shutdown():
        bridge.set_mode('manual')
        mode, arm = bridge.arm_throttle(False)
        if mode == 'MANUAL' and not arm:
            break
        rospy.sleep(0.5)
