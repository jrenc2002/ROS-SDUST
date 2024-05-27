#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 真正的主文件

import rospy
import cv2
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from traffic_light_detection import TrafficLightDetector
from zebra_crossing_detection import ZebraCrossingDetector
from lane_detection2 import LaneDetector
from car_control import CarController
from arrow_detection import ArrowDetector
from lane_detection import LineFollower
from obstacle_detection import ObstacleDetector
import os

class ImageProcessorROS:
    def __init__(self, car_controller):
        rospy.init_node('smartcar')
        self.bridge = CvBridge()
        self.car_controller = car_controller
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # 发布处理后的图像
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            # 处理图像，获取速度和偏移量
            linear_vel, angular_vel = main_control(self.car_controller, cv_image)
            # 发布速度控制指令
            self.publish_velocity(linear_vel, angular_vel)
        except Exception as e:
            rospy.logerr(e)

    # 发布速度和偏移角度
    def publish_velocity(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)


def main_control(car_controller, image):
    # 图像处理
    # masked_image = image_process(image)

    # 红绿灯检测
    light_detector = TrafficLightDetector()
    light_status = light_detector.detect_light(image)

    # 斑马线检测
    zebra_detector = ZebraCrossingDetector()
    zebra_status = zebra_detector.detect_zebra_crossing(image)

    # 加速箭头检测
    arrow_detector = ArrowDetector()
    arrow_status = arrow_detector.detect_arrow(image)

    # 障碍物检测
    # obstacle_detector = ObstacleDetector()
    # obstacle_status = obstacle_detector.detect_obstacle(image)

    # # 车道线检测
    # lane_detector = LaneDetector()
    # lane_err = lane_detector.detect_lane(image)
    # 车道线检测
    lineFollower = LineFollower()
    status, offset, slope = lineFollower.detect_lane(image)

    # 更新参数，进行小车控制
    # linear_vel, angular_vel = car_controller.control(light_status, zebra_status, arrow_status, lane_err)
    # 更新参数，进行小车控制
    linear_vel, angular_vel = car_controller.control(light_status, zebra_status, arrow_status, status, offset, slope)
    return linear_vel, angular_vel


if __name__ == '__main__':
    # 初始化车辆控制对象
    car_controller = CarController()

    # 启动本地显示节点
    viewer_script_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'image_viewer.py')
    subprocess.Popen(['python3', viewer_script_path])

    # 启动图像处理节点
    ImageProcessorROS(car_controller)
