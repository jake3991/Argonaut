#!/usr/bin/env python
"""
BlueRov video capture class
"""

import cv2
import gi
import numpy as np

import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv_bridge

pub_info = True
try:
    import camera_info_manager
except ImportError as e:
    rospy.logerr('camera_info_manager_py not installed')
    pub_info = False


gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (caps.get_structure(0).get_value('height'),
             caps.get_structure(0).get_value('width'), 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst([
            self.video_source, self.video_codec, self.video_decode,
            self.video_sink_conf
        ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK


if __name__ == '__main__':
    rospy.init_node("bluerov_video_node")
    # Create the video object
    # Add port= if is necessary to use a different one
    video = Video(4777)
    bridge = cv_bridge.CvBridge()
    img_pub = rospy.Publisher('/camera/image', Image, queue_size=10)

    if pub_info:
        info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
        info_manger = camera_info_manager.CameraInfoManager(cname='camera', namespace='camera')
        info_manger.loadCameraInfo()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Wait for the next frame
        if not video.frame_available():
            continue

        frame = video.frame()

        img_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.frame_id = 'camera'
        img_msg.header.stamp = rospy.Time.now()
        img_pub.publish(img_msg)

        if pub_info:
            info_msg = info_manger.getCameraInfo()
            info_msg.header.frame_id = img_msg.header.frame_id
            info_msg.header.stamp = img_msg.header.stamp
            info_msg.width = frame.shape[1]
            info_msg.height = frame.shape[0]
            info_pub.publish(info_msg)

        rate.sleep()
