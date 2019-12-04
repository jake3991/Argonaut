#!/usr/bin/env python
import time
import curses

import rospy
from rostopic import ROSTopicHz
from rti_dvl.msg import BottomTrack
from sensor_msgs.msg import Imu
from bar30_depth.msg import Depth
from sonar_oculus.msg import OculusPing

from bluerov_bridge import Bridge


OCULUS_PARTNUMBER = {0: 'NAN', 1032: 'M750d', 1042: 'M1200d'}

dvl, depth, imu, ping = BottomTrack(), Depth(), Imu(), OculusPing()
hz = ROSTopicHz(-1)
dvl_hz, depth_hz, imu_hz, sonar_hz = 0, 0, 0, 0


def dvl_callback(msg):
    global dvl
    dvl = msg
    hz.callback_hz(msg, 'dvl')


def depth_callback(msg):
    global depth
    depth = msg
    hz.callback_hz(msg, 'depth')


def imu_callback(msg):
    global imu
    imu = msg
    hz.callback_hz(msg, 'imu')


def sonar_callback(msg):
    global ping
    ping = msg
    hz.callback_hz(msg, 'sonar')


if __name__ == "__main__":
    rospy.init_node('bluerov_dashboard')

    device = 'udp:192.168.2.1:14553'
    while not rospy.is_shutdown():
        try:
            rov = Bridge(device)
        except socket.error:
            rospy.logerr(
                'Failed to make mavlink connection to device {}'.format(
                    device))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)
    rov.update()

    dvl_sub = rospy.Subscriber('/rti/bottom_tracking/raw', BottomTrack, dvl_callback, queue_size=100)
    depth_sub = rospy.Subscriber('/bar30/depth/raw', Depth, depth_callback, queue_size=100)
    imu_sub = rospy.Subscriber('/vn100/imu/raw', Imu, imu_callback, queue_size=1000)
    sonar_sub = rospy.Subscriber('/sonar_oculus_node/ping', OculusPing, sonar_callback, queue_size=10)

    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    curses.curs_set(0)

    rospy.on_shutdown(curses.endwin)

    stdscr.addstr(2, 5, 'BlueROV2 Dashboard')

    n, rate = 0, rospy.Rate(10)
    while not rospy.is_shutdown():
        rov.update()

        voltage, current = rov.get_battery()
        mode, arm = rov.get_mode()
        arm = 'ARM' if arm else 'DISARM'

        cx, cy, cz, cr = rov.get_cmd_vel()

        water = rospy.get_param('/water', 'NAN')

        # 1 Hz
        if n % 10 == 0:
            ret = hz.get_hz('dvl')
            dvl_hz = ret[0] if ret else 0.0
            ret = hz.get_hz('depth')
            depth_hz = ret[0] if ret else 0.0
            ret = hz.get_hz('imu')
            imu_hz = ret[0] if ret else 0.0
            ret = hz.get_hz('sonar')
            sonar_hz = ret[0] if ret else 0.0
        n += 1

        stdscr.addstr( 4, 5, '======== Vehicle ============')
        stdscr.addstr( 5, 5, 'Time       |  {:>15s}'.format(time.strftime('%H:%M:%S', time.gmtime())))
        stdscr.addstr( 6, 5, 'Battery    |       {:04.1f}V {:03.1f}A'.format(voltage, current))
        stdscr.addstr( 7, 5, 'Mode       |  {:>8s} {:>6s}'.format(mode, arm))
        stdscr.addstr( 8, 5, '=============================')

        stdscr.addstr(10, 5, '======== RC_CHANNELS ========')
        stdscr.addstr(11, 5, 'Forward    |  {:>15d}'.format(cx))
        stdscr.addstr(12, 5, 'Lateral    |  {:>15d}'.format(cy))
        stdscr.addstr(13, 5, 'Throttle   |  {:>15d}'.format(cz))
        stdscr.addstr(14, 5, 'Yaw        |  {:>15d}'.format(cr))
        stdscr.addstr(15, 5, '=============================')

        stdscr.addstr(17, 5, '======== Sensor Rate ========')
        stdscr.addstr(18, 5, 'DVL        | {:>13.1f} Hz'.format(dvl_hz))
        stdscr.addstr(19, 5, 'Depth      | {:>13.1f} Hz'.format(depth_hz))
        stdscr.addstr(20, 5, 'IMU        | {:>13.1f} Hz'.format(imu_hz))
        stdscr.addstr(21, 5, 'Sonar      | {:>13.1f} Hz'.format(sonar_hz))
        stdscr.addstr(22, 5, '=============================')

        stdscr.addstr(24, 5, '======== Environment ========')
        stdscr.addstr(25, 5, 'Depth      | {:>14.1f} m'.format(depth.depth))
        stdscr.addstr(26, 5, 'Altitude   | {:>14.1f} m'.format(dvl.altitude))
        stdscr.addstr(27, 5, 'Water      | {:>16s}'.format(water))
        stdscr.addstr(28, 5, '=============================')

        stdscr.addstr(30, 5, '======== Sonar ==============')
        stdscr.addstr(31, 5, 'Mode       | {:>16s}'.format(OCULUS_PARTNUMBER[ping.part_number]))
        stdscr.addstr(32, 5, 'Salinity   | {:>16.1f}'.format(ping.fire_msg.salinity))
        stdscr.addstr(33, 5, 'Gain       | {:>16.1f}'.format(ping.fire_msg.gain))
        stdscr.addstr(34, 5, 'Range      | {:>16.1f}'.format(ping.fire_msg.range))
        stdscr.addstr(35, 5, '=============================')

        stdscr.addstr(36, 5, '======== Velocity ===========')
        stdscr.addstr(37, 5, 'X          | {:>12.2f} m/s'.format(dvl.velocity.x))
        stdscr.addstr(38, 5, 'Y          | {:>12.2f} m/s'.format(dvl.velocity.y))
        stdscr.addstr(39, 5, 'Z          | {:>12.2f} m/s'.format(dvl.velocity.z))
        stdscr.addstr(40, 5, '=============================')

        stdscr.refresh()
        rate.sleep()
    
    curses.nocbreak(); stdscr.keypad(0); curses.echo()
    curses.endwin()
    