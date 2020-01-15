#!/usr/bin/env python
"""
Purpose of the file: subscribe to a topic called /image_raw
of type sensor_msgs/Image
Apply filter to the resulting image
"""
from __future__ import print_function
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class SubThenFilter:
    def __init__(self):
        self.sub = rospy.Subscriber(
            inputTopic, Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher(ouputTopic, Image, queue_size=1)
        self.bridge = CvBridge()
        self.median_blur_size = blurSize
        self.use_median_blur = self.median_blur_size > 0

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
            return

        cv_image.setflags(write=1)

        # Clip value between minrange and maxrange
        # cv_image = np.clip(cv_image, a_min = 450, a_max = 2800)
        minrange = 450
        maxrange = 6000
        cv_image[(cv_image > maxrange) | (cv_image < minrange)] = 0

        # cv_image = np.nan_to_num(cv_image)
        if self.use_median_blur:

            # The first parameter is the original image,
            # kernel is the matrix with which image is
            # convolved and third parameter is the number
            # of iterations, which will determine how much
            # you want to erode/dilate a given image.
            kernel = getKernel()
            erodeImage = cv2.erode(cv_image, kernel, iterations=2)
            cv_image = cv2.dilate(erodeImage, kernel, iterations=2)

	    cv_image = cv2.medianBlur(cv_image, self.median_blur_size)

        try:
            msg = self.bridge.cv2_to_imgmsg(cv_image, "passthrough")
            data.data = msg.data
            self.pub.publish(data)

        except CvBridgeError as e:
            print(e)


def getKernel():
    kernel = np.ones((3, 3), np.uint8)

    kernel[0, 0] = 0
    kernel[0, 2] = 0
    kernel[2, 0] = 0
    kernel[2, 2] = 0


if __name__ == "__main__":
    inputTopic = rospy.get_param(
        rospy.search_param('depth_filter/input_topic'))
    ouputTopic = rospy.get_param(
        rospy.search_param('depth_filter/output_topic'))
    blurSize = rospy.get_param(rospy.search_param('depth_filter/blur_size'))

    rospy.init_node("depth_filter", anonymous=True)

    sf = SubThenFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

cv2.destroyAllWindows()
