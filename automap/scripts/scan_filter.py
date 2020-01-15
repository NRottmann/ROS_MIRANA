#!/usr/bin/env python

from __future__ import print_function
import cv2
import scipy.signal as signal
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np


class SubThenFilter:
    def __init__(self):
        self.sub = rospy.Subscriber(
            inputTopic, LaserScan, self.scanCallback, queue_size=1)
        self.pub = rospy.Publisher(ouputTopic, LaserScan, queue_size=1)
        self.median_blur_size = blurSize
        self.use_median_blur = self.median_blur_size > 0

    def scanCallback(self, data):
        x = np.array(signal.medfilt(data.ranges, kernel_size=31))
        x = np.nan_to_num(x)
        x[x == 0] = 4.0
        data.ranges = x
        self.pub.publish(data)


if __name__ == "__main__":
    inputTopic = rospy.get_param(rospy.search_param('scan_filter/input_topic'))
    ouputTopic = rospy.get_param(
        rospy.search_param('scan_filter/output_topic'))
    blurSize = rospy.get_param(rospy.search_param('scan_filter/blur_size'))

    rospy.init_node("scan_filter", anonymous=True)

    sf = SubThenFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

cv2.destroyAllWindows()
