#!/usr/bin/env python
import sys
import rospy
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from rti_dvl.msg import DVL, Command, BottomTrack

import pynmea2
import serial

SALINITY = {'fresh': 0, 'salt': 35}


class SyncTime(object):
    """Time translator
    - See https://github.com/ethz-asl/cuckoo_time_translator for more details.
    - Implement https://github.com/ros-drivers/urg_node/blob/d2722c60f1b4713bbe1d39f32849090dece0104d/src/urg_c_wrapper.cpp#L1052
    """

    def __init__(self):
        self.hardware_clock = 0.0
        self.last_hardware_time_stamp = 0.0
        self.hardware_clock_adj = 0.0
        self.adj_count = 0
        self.adj_alpha = 0.01

    def sync(self, time_stamp, system_time_stamp):
        delta = time_stamp - self.last_hardware_time_stamp
        self.hardware_clock += delta
        cur_adj = system_time_stamp.to_sec() - self.hardware_clock
        if self.adj_count > 0:
            self.hardware_clock_adj = (
                self.adj_alpha * cur_adj + (1.0 - self.adj_alpha) * self.hardware_clock_adj)
        else:
            self.hardware_clock_adj = cur_adj

        self.adj_count += 1
        self.last_hardware_time_stamp = time_stamp

        stamp = system_time_stamp
        if self.adj_count > 100:
            stamp = stamp.from_sec(self.hardware_clock + self.hardware_clock_adj)

            if abs((stamp - system_time_stamp).to_sec()) > 0.1:
                self.adj_count = 0
                self.hardware_clock = 0.0
                self.last_hardware_time_stamp = 0.0
                self.stamp = system_time_stamp
        return stamp


def write_to_dvl(s, duration=0.5):
    dvl.write((s.strip() + '\r'))
    
    rospy.sleep(duration)
    


if __name__ == '__main__':
    rospy.init_node('rti_dvl_node')

    dev = rospy.get_param('~dev', '/dev/ttyUSB0')
    # Get configurations split by ;
    config = rospy.get_param('~commands', '').split(';')
    #print config

    if not rospy.has_param('/water'):
        rospy.set_param('/water', 'fresh')
    water = rospy.get_param('/water', 'fresh')
    salinity = SALINITY[water]

    dvl_pub = rospy.Publisher('/rti/body_velocity/raw', DVL, queue_size=100)
    bt_pub = rospy.Publisher('/rti/bottom_tracking/raw', BottomTrack, queue_size=100)

    while not rospy.is_shutdown():
        try:
            rospy.loginfo('Open device {}'.format(dev))
            #dvl = serial.Serial(dev, baudrate = 115200, dsrdtr=True, rtscts=True, timeout=1.0)
            dvl = serial.Serial(dev,baudrate = 115200,timeout = 1.0)
            rospy.sleep(1.0)
        except serial.SerialException:
            rospy.logerr('Fail to open device {}'.format(dev))
            rospy.sleep(1.0)
        else:
            break
    if rospy.is_shutdown():
        sys.exit(-1)

    # STOP
    while dvl.in_waiting:
        line = dvl.readline()
	print line

        if line.strip().startswith('STOP'):
            break
        write_to_dvl('STOP')
	#write_to_dvl('STOP')
	#write_to_dvl('STOP')

        rospy.loginfo_throttle(1.0, 'Trying to stop DVL pinging...')

    # Zero pressure sensor
    write_to_dvl('CPZ')
    # Change water salinity
    write_to_dvl('CWS {}'.format(salinity))
    # Write extra commands in config file
    for c in config:
        if c:
            write_to_dvl(c)
            rospy.loginfo('Write to DVL: ' + c)

    cmd = Command()

    line = dvl.readline()

    cp = line.strip().split(' ', 1)
    print cp

    while True:
        write_to_dvl('CSHOW')
        rospy.loginfo_throttle(1.0, 'Trying to read DVL commands...')


        received = False
        while dvl.in_waiting:
	    print "test"
            line = dvl.readline()

            cp = line.strip().split(' ', 1)
            if len(cp) == 2:
                c, p = cp
		print c
                if c == 'CEPO':
                    received = True
                c = c.split('[', 1)[0]
                if c in cmd.__slots__:
                    setattr(cmd, c, p)
        
        if received:
            break
        else:
            rospy.sleep(1.0)

    # Start
    while not dvl.in_waiting:
        write_to_dvl('START')
        rospy.loginfo_throttle(1.0, 'Trying to start DVL pinging...')

    sync = SyncTime()
    reader = pynmea2.NMEAStreamReader(errors='ignore')

    bt = None
    while not rospy.is_shutdown():
        try:
            char = dvl.read()
        except serial.SerialException:
            break
        for msg in reader.next(char):
            if isinstance(msg, pynmea2.types.rti.RTI01) or isinstance(msg, pynmea2.types.rti.RTI03):
                stamp = sync.sync(msg.time / 100.0, rospy.Time.now())

                # For back compability
                d = DVL()
                d.header.stamp = stamp
                d.velocity.x = msg.x / 1000.0
                d.velocity.y = msg.y / 1000.0
                d.velocity.z = msg.z / 1000.0
                d.temperature = msg.temperature / 100.0
                d.altitude = msg.depth / 1000.0
                d.time = msg.time / 100.0
                dvl_pub.publish(d)

                bt = BottomTrack()
                bt.header.stamp = stamp
                bt.command = cmd
                bt.velocity.x = msg.x / 1000.0
                bt.velocity.y = msg.y / 1000.0
                bt.velocity.z = msg.z / 1000.0
                bt.temperature = msg.temperature / 100.0
                bt.altitude = msg.depth / 1000.0
                bt.time = msg.time / 100.0
                bt.sample = msg.number

            if isinstance(msg, pynmea2.types.rti.RTI30) or isinstance(msg, pynmea2.types.rti.RTI32):
                if bt is None:
                    continue
                bt.orientation.x = msg.roll
                bt.orientation.y = msg.pitch
                bt.orientation.z = msg.heading
                bt_pub.publish(bt)

    dvl.close()
