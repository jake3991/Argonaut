#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
import rosbag
from rti_dvl.msg import DVL
from sensor_msgs.msg import Imu
from bar30_depth.msg import Depth
from tf import transformations

plt.style.use('seaborn-talk')
DVL_TOPIC = '/rti/body_velocity/raw'
IMU_TOPIC = '/vn100/imu/raw'
DEPTH_TOPIC = '/bar30/depth/raw'


def read_bag(file, start=None, duration=None):
    bag = rosbag.Bag(file)
    start = start if start is not None else 0
    start_time = bag.get_start_time() + start
    end_time = bag.get_end_time()
    if duration is None or duration < 0 or duration == float("inf"):
        duration = end_time - start_time
    else:
        end_time = start_time + duration

    for topic, msg, t in bag.read_messages(
        topics=[IMU_TOPIC, DVL_TOPIC, DEPTH_TOPIC],
        start_time=rospy.Time.from_sec(start_time),
        end_time=rospy.Time.from_sec(end_time),
    ):
        yield topic, msg

    bag.close()


def load_nav_data(file, start=0, duration=None):
    dvl, depth, imu = [], [], []
    for topic, msg in read_bag(file, start, duration):
        time = msg.header.stamp.to_sec()
        if topic == DVL_TOPIC:
            dvl.append(
                (time, msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.altitude)
            )
        elif topic == DEPTH_TOPIC:
            depth.append((time, msg.depth))
        elif topic == IMU_TOPIC:
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            wx = msg.angular_velocity.x
            wy = msg.angular_velocity.y
            wz = msg.angular_velocity.z
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            # IMU is -roll90
            q = transformations.quaternion_matrix((qx, qy, qz, qw))
            r = q.dot(transformations.euler_matrix(0, 0, np.pi / 2, 'szyx'))
            y, p, r = transformations.euler_from_matrix(r, 'szyx')
            # Hardware time
            t = msg.linear_acceleration_covariance[0]
            imu.append((time, ax, ay, az, wx, wy, wz, r, p, y, t))

    dvl = np.array(dvl)
    depth = np.array(depth)
    imu = np.array(imu)
    t0 = [a[0, 0] for a in (dvl, depth, imu) if len(a)]
    if not t0:
        return None, None, None
    else:
        t0 = min(t0)

    if len(dvl):
        dvl[:, 0] -= t0
    if len(imu):
        imu[:, 0] -= t0
        imu[:, -1] -= imu[0, -1]
    if len(depth):
        depth[:, 0] -= t0
    return dvl, depth, imu


def plot_nav_data(dvl, depth, imu, max_vel=1.0):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(15, 8), dpi=200)
    ax1.plot(dvl[:, 0], dvl[:, 1], 'r', label='vx', alpha=0.7)
    ax1.plot(dvl[:, 0], dvl[:, 2], 'g', label='vy', alpha=0.7)
    ax1.plot(dvl[:, 0], dvl[:, 3], 'b', label='vz', alpha=0.7)

    vel = dvl[:, 1:4].ravel()
    vmin = np.min(vel[(vel < 0) & (vel > -max_vel)]) - 0.1
    vmax = np.max(vel[(vel > 0) & (vel < max_vel)]) + 0.1
    ax1.set_ylim(-vmax, vmax)
    ax1.legend(fontsize='x-large', loc='upper right')
    ax1.grid(axis='y')
    ax1.set_ylabel('m/s')

    ax2.plot(dvl[:, 0], dvl[:, 4], 'k', label='altitude', alpha=0.7)
    ax2.plot(depth[:, 0], depth[:, 1], 'c', label='depth', alpha=0.7)

    dmax = max(np.max(dvl[dvl[:, 4] > 0, 4]), np.max(depth[:, 1])) + 0.1
    dmin = min(np.min(dvl[dvl[:, 4] > 0, 4]), np.max(depth[:, 1])) - 0.1
    ax2.set_ylim(dmin, dmax)
    ax2.legend(fontsize='x-large', loc='upper right')
    ax2.grid(axis='y')
    ax2.set_ylabel('m')

    ax3.plot(imu[:, 0], np.rad2deg(imu[:, 7]), 'k', alpha=0.7, label='roll')
    ax3.plot(imu[:, 0], np.rad2deg(imu[:, 8]), 'c', alpha=0.7, label='pitch')
    ax3.legend(fontsize='x-large', loc='upper right')
    ax3.grid(axis='y')
    ax3.set_ylabel('deg')
    plt.tight_layout()


if __name__ == "__main__":
    import sys
    bagname = sys.argv[1]
    dvl, depth, imu = load_nav_data(bagname)
    plot_nav_data(dvl, depth, imu)
    plt.show()

