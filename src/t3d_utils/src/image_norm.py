#! /usr/bin/env python
from __future__ import print_function

from numpy.testing.utils import _gen_alignment_data

import cv_bridge
import rospy                                          
import cv2
from std_msgs.msg import Float32
from std_msgs.msg import UInt16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

pub = rospy.Publisher('/seek/image1', Image, queue_size=10) 
pubImageRawMean = rospy.Publisher('/seek/image_raw_mean', Float32, queue_size=10) 
bridge = CvBridge()

# seek_raw_mean_path = '/home/mpikh/data/thermo3d/seek_raw_mean7.png'
# frame_mean = cv2.imread(seek_raw_mean_path,-cv2.IMREAD_ANYDEPTH)

seek_raw_mean_path = '/home/mpikh/data/thermo3d/seek_raw_mean14.txt'
frame_mean = np.loadtxt(seek_raw_mean_path)

# cv2.namedWindow('seek', cv2.WINDOW_NORMAL)

# fr_min_all = 100000
# fr_max_all = 0
temper = -1
ff = 0.95

def callbackTemper(msg):
    # print(msg.data)
    global temper
    if (temper < 0):
        temper = 1.0 * msg.data
    else:
        temper = ff*temper + (1-ff)*msg.data
    print("device temperature:")
    print(temper)

def callback(msg):
    global frame_mean
    frame_raw1 = bridge.imgmsg_to_cv2(msg,'passthrough')
    # temper_bias = -7.59573770e+04 + 1.23870778e+01*temper
    # temper_bias = -5.3e4 + 9.3*temper
    # temper_bias = -1.74645842e+04 + 4.46525211e+00*temper
    # temper_bias = -3.94576264e+04 + 7.45183458e+00*temper
    # temper_bias = -4.86516629e+04 + 8.19877559e+00*temper
    temper_bias =  1.49939414e+04 +  1.66974434e-02*temper
    # temper_bias = c[1] + c[0]*temper
    frame_raw = frame_raw1.astype(np.float32) - frame_mean.astype(np.float32)
    # frame_raw = frame_raw1.astype(np.float32)
    frame_raw -= temper_bias
    fr_min = np.min(frame_raw.flatten())
    fr_max = np.max(frame_raw.flatten())
    pubImageRawMean.publish(np.mean(frame_raw.flatten().astype(np.float32)))
    # if (fr_min < fr_min_all):
    #     fr_min_all = fr_min
    # if (fr_max > fr_max_all):
    #     fr_max_all = fr_max
    # print(fr_min,fr_max,fr_min_all,fr_max_all)
    # print(temper,temper_bias)
    print(fr_min,fr_max)
    vl = -200
    vu = 200
    frame = 255.0 * (frame_raw - vl) / (vu - vl)
    frame[frame < 0] = 0
    frame[frame > 255] = 255
    frame = frame.astype(np.uint8)


    #cv::normalize(frame, grey_frame, 0, 65535, cv::NORM_MINMAX);
    # frame = cv2.normalize(frame_raw, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    # frame = cv2.convertScaleAbs(frame)
    # cv2.imshow('seek',frame)
    # cv2.waitKey(1)
    frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
    pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    # cv2.imshow('seek',frame)
    # cv2.waitKey(1)

    # # print msg.data

rospy.init_node('image_norm')
sub = rospy.Subscriber('/seek/image', Image, callback)
sub = rospy.Subscriber('/seek/device_temperature_raw', UInt16, callbackTemper)
rospy.spin()