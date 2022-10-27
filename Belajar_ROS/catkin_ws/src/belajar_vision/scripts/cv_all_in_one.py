#!/usr/bin/env python3
# Basics ROS program to publish real-time streaming 
# video from your built-in webcam
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
# from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
# from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import gi
import numpy as np
import math

# Class Video BlueRov untuk mengambil video dari gstreamer udp
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

# Fungsi kosong untuk TrackBar
def nothing(x):
    pass

# Callback hsv subscriber
def hsv_sub(val):
    cv2.setTrackbarPos('HMin', 'HSV', val.data[0])
    cv2.setTrackbarPos('SMin', 'HSV', val.data[1])
    cv2.setTrackbarPos('VMin', 'HSV', val.data[2])
    cv2.setTrackbarPos('HMax', 'HSV', val.data[3])
    cv2.setTrackbarPos('SMax', 'HSV', val.data[4])
    cv2.setTrackbarPos('VMax', 'HSV', val.data[5])
    rospy.loginfo("HSV subscribed")

def clamp(value):
    return max(min(value, 1), -1)

# Callback Scanning subscriber
scanning = False
def scan(val):
    global scanning 
    scanning = val

# Fungsi utama
def run():
    # Inisiasi Node, anon => artinya unique
    rospy.init_node('cv_all_in_one', anonymous=True)

    # Subscriber dan Publisher
    rospy.Subscriber('hsv_pub', Int32MultiArray, hsv_sub)
    rospy.Subscriber('scan_pub', Bool, scan)
    center_pub = rospy.Publisher('center_pub', Pose, queue_size=10)
    distance_pub = rospy.Publisher('distance_pub', Pose, queue_size=10)
        
    # Go through the loop 60 times per second, a.k.a 60fps dalam kasus video
    rate = rospy.Rate(60) # 10hz
        
    # Create a VideoCapture object
    cap = Video()

    # Inisiasi Window untuk monitoring OPENCV
    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 400, 200)

    cv2.namedWindow('Live Leak (ಠಿ ͟ʖ ͡ಠ)', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Deep Web ⊙д⊙', cv2.WINDOW_NORMAL)

    cv2.createTrackbar('HMin', 'HSV', 0, 179, nothing)
    cv2.createTrackbar('SMin', 'HSV', 0, 255, nothing)
    cv2.createTrackbar('VMin', 'HSV', 0, 255, nothing)
    cv2.createTrackbar('HMax', 'HSV', 179, 179, nothing)
    cv2.createTrackbar('SMax', 'HSV', 255, 255, nothing)
    cv2.createTrackbar('VMax', 'HSV', 255, 255, nothing)
    
    # While ROS is still running.
    while not rospy.is_shutdown():

        # Skip loop jika frame tidak tersedia
        if not cap.frame_available():
            continue
        # Mengambil frame dari video
        frame = cap.frame()
        # Mengambil dimensi dari frame
        (h, w) = frame.shape[:2]

        # Get current positions of all trackbars
        hMin = cv2.getTrackbarPos('HMin', 'HSV')
        sMin = cv2.getTrackbarPos('SMin', 'HSV')
        vMin = cv2.getTrackbarPos('VMin', 'HSV')
        hMax = cv2.getTrackbarPos('HMax', 'HSV')
        sMax = cv2.getTrackbarPos('SMax', 'HSV')
        vMax = cv2.getTrackbarPos('VMax', 'HSV')

        # Set up lower dan upper untuk HSV
        lower = np.array([hMin, sMin, vMin])
        upper = np.array([hMax, sMax, vMax])
        
        # Convert dari BGR ke HSV, karena HSV lebih baik dalam object detection
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Masking frame berdasarkan lower hsv dan upper hsv
        mask = cv2.inRange(hsv, lower, upper)

        # Binary frame berdasarkan mask
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Mengambil Value Channel, a.k.a langsung grayscale tanpa perlu convert
        (hChannel, sChannel, vChannel) = cv2.split(result)

        # Threshold warna putih untuk blob
        ret,thresh = cv2.threshold(vChannel,127,255,0)

        # Menggunakan moments untuk menentukan centroid object, pilihan general untuk segala shape. Bila spesifik dapat menggunakan metode lain
        M = cv2.moments(thresh)

        # Jika sedang mode scanning
        if(ret and scanning):
            # calculate x,y coordinate of center
            # Menghindari division by zero
            if(M["m00"] != 0):
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Menghitung jarak antar kedua titik ( Tengah video dan centroid )
                d = math.sqrt((cX - w/2) ** 2 + (cY - h/2) ** 2)

                # Melingkari titik centroid
                cv2.circle(result, (cX, cY), 5, (255, 255, 255), -1)
                cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)

                # Inisialisasi untuk publisher center_pub, dimana akan berisikan normalized vector
                center = Pose()
                # center.position.x = (cX - w/2) / abs((cX - w/2)) if (cX - w/2 != 0) else cX - w/2
                # center.position.y = (cY - h/2) / abs((cY - h/2)) if (cY - h/2 != 0) else cY - h/2
                center.position.x = clamp(cX - w/2)
                center.position.y = clamp(cY - h/2)
                center_pub.publish(center)

                # Inisialisasi publisher yang akan berisikan jarak ke titik center, dalam axis x, y serta jarak antar dua titik
                distance = Pose()
                distance.position.x = cX - w/2
                distance.position.y = cY - h/2
                distance.position.z = d
                distance_pub.publish(distance)

                # Membuat garis dari kedua titik
                cv2.line(result, (int(w/2), int(h/2)), (int(cX), int(cY)), (127, 255, 0), 1)
                cv2.line(frame, (int(w/2), int(h/2)), (int(cX), int(cY)), (127, 255, 0), 1)

        # Membuat lingkaran pada tengah frame video
        cv2.circle(result, (int(w/2), int(h/2)), 10, (0, 255, 0), 2)
        cv2.circle(frame, (int(w/2), int(h/2)), 10, (0, 255, 0), 2)

        # 320 x 240 frame size

        # Menampilkan frame-frame yang sudah diedit
        cv2.imshow("Live Leak (ಠಿ ͟ʖ ͡ಠ)", frame)
        cv2.imshow("Deep Web ⊙д⊙", result)
        cv2.waitKey(1)
                
        # Sleep just enough to maintain the desired rate
        rate.sleep()
         
if __name__ == '__main__':
  try:
    run()
  except rospy.ROSInterruptException:
    pass
