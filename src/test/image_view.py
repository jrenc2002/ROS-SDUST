#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 本地显示图像的脚本

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageViewer:
    def __init__(self):
        rospy.init_node('image_viewer')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/processed_image', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 显示图像
            cv2.imshow("Processed Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(e)

if __name__ == '__main__':
    ImageViewer()
