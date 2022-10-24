#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import gi
import numpy as np

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
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None

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
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK
 
def nothing(x):
    pass

def hsv_sub(val):
    cv2.setTrackbarPos('HMin', 'HSV', val.data[0])
    cv2.setTrackbarPos('SMin', 'HSV', val.data[1])
    cv2.setTrackbarPos('VMin', 'HSV', val.data[2])
    cv2.setTrackbarPos('HMax', 'HSV', val.data[3])
    cv2.setTrackbarPos('SMax', 'HSV', val.data[4])
    cv2.setTrackbarPos('VMax', 'HSV', val.data[5])
    rospy.loginfo("HSV subscribed")


def run():
 
    # Node is publishing to the video_frames topic using 
    # the message type Image
    #   pub = rospy.Publisher('video_frames', Image, queue_size=10)
        
    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node('cv_all_in_one', anonymous=True)
    rospy.Subscriber('hsv_pub', Int32MultiArray, hsv_sub)
        
    # Go through the loop 10 times per second
    rate = rospy.Rate(60) # 10hz
        
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    # cap = cv2.VideoCapture(0)
    # cap = cv2.VideoCapture("udpsrc port=5600 ! application/x-rtp,payload=96,encoding-name=H264 ! rtpjitterbuffer mode=1 ! rtph264depay ! h264parse ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER);
    cap = Video()

    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 400, 300)

    cv2.namedWindow('Live Leak (ಠಿ ͟ʖ ͡ಠ)', cv2.WINDOW_NORMAL)

    cv2.createTrackbar('HMin', 'HSV', 0, 179, nothing)
    cv2.createTrackbar('SMin', 'HSV', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'HSV', 0, 255, nothing)
    cv2.createTrackbar('HMax', 'HSV', 179, 179, nothing)
    cv2.createTrackbar('SMax', 'HSV', 255, 255, nothing)
    cv2.createTrackbar('VMax', 'HSV', 255, 255, nothing)
    
    # While ROS is still running.
    while not rospy.is_shutdown():
        if not cap.frame_available():
            continue
     
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        frame = cap.frame()
        (h, w) = frame.shape[:2]

        # Get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'HSV')
        sMin = cv2.getTrackbarPos('SMin', 'HSV')
        vMin = cv2.getTrackbarPos('VMin', 'HSV')
        hMax = cv2.getTrackbarPos('HMax', 'HSV')
        sMax = cv2.getTrackbarPos('SMax', 'HSV')
        vMax = cv2.getTrackbarPos('VMax', 'HSV')
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
        
        # Print debugging information to the terminal
        # Convert to HSV format and color threshold
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.circle(result, (int(w/2), int(h/2)), 15, (0, 255, 0), 2)

        # calculate moments of binary image]
        gray_image = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray_image,127,255,0)
        M = cv2.moments(thresh)
        
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv2.circle(result, (cX, cY), 5, (255, 255, 255), -1)
        
        cv2.imshow("Live Leak (ಠಿ ͟ʖ ͡ಠ)", result)
        cv2.waitKey(1)
                
        # Sleep just enough to maintain the desired rate
        rate.sleep()
         
if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
