# 没有模块化的主代码
# -*- coding: UTF-8 -*-
# !/usr/bin/env python
#  导入所需库
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math
import numpy as np


class Follower:
    def __init__(self):
        #  cv_bridge将ROS重点Imgage类型转换为opencv图像格式
        self.bridge = cv_bridge.CvBridge()
        #  订阅相机驱动（ROS）发布的Image可见光图像数据，
        #  每收到一帧图像，用image_callback回调函数进行处理
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        #  发布速度控制指令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # 对红灯出现次数计数
        self.red_cnt = 0
        self.zebra_judge = 0
        self.zebra_cnt = 0

    #  图像处理回调函数
    def image_callback(self, msg):
        #  将ROS下图像格式转为opencv图像格式
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 定义两个核	（kernel_Ero用于腐蚀，kernel_Dia用于膨胀）
        kernel_Ero = np.ones((15, 3), np.uint8)
        kernel_Dia = np.ones((15, 3), np.uint8)
        # 斑马线:判断到斑马线f=1，没有判断到就等于0
        # ----------------------------------------------------------------
        # 将复制的图像裁剪为480*360
        copy_img = cv2.resize(image, (480, 360))
        # 灰度值转换
        imgGray = cv2.cvtColor(copy_img, cv2.COLOR_BGR2GRAY)
        # 高斯滤波去噪
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 0)
        # 阈值处理
        ret, thresh = cv2.threshold(imgBlur, 200, 255, cv2.THRESH_BINARY)
        # 腐蚀
        imgEro = cv2.erode(thresh, kernel_Ero, iterations=2)
        # 膨胀
        imgDia = cv2.dilate(imgEro, kernel_Dia, iterations=4)

        # 感兴趣区间
        h = imgDia.shape[0]
        w = imgDia.shape[1]
        poly = np.array([
            [(0, 360), (0, 240), (640, 240), (640, 360)]
        ])
        mask = np.zeros_like(imgDia)

        cv2.fillPoly(mask, poly, 255)

        masked_image = cv2.bitwise_and(imgDia, mask)
        # cv2.imshow("111", masked_image)
        # 轮廓检测
        _, contouts, hie = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = contouts

        xx = 0
        for i in cnt:

            # 坐标赋值
            x, y, w, h = cv2.boundingRect(i)
            # #roi位置判断
            # if y>50 and y<450 and x<1200 and w>10 and h>10:
            # 打印图像轮廓的面积
            # print("cv2.contourArea(i) = " + str(cv2.contourArea(i)))
            if cv2.contourArea(i) > 1000 and cv2.contourArea(i) < 7000 and h > w:
                print(cv2.contourArea(i))
                xx += 1
                # 画出轮廓
                # cv2.drawContours(copy_img, i, -1, (0, 255, 0), 2)
                # rect = cv2.minAreaRect(i)
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                # cv2.drawContours(copy_img, [box], 0, (0, 0, 255), 2)
        if xx > 2:
            print("banma: ", xx)
            print("slow down*****************************************")
            self.zebra_judge = 1
        else:
            self.zebra_judge = 0
        # ----------------------------------------------------------------
        # follower judge
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 0, 95])
        upper_hsv = np.array([180, 255, 255])
        imgDia = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
        h, w, d = image.shape
        image1 = imgDia[:, 0:int(w / 2)]
        image2 = imgDia[:, int(w / 2):w]
        #  获取图像的高度（h）和宽度（w）
        h, w1 = image1.shape
        h, w2 = image2.shape
        #  设置车道线搜索区域
        search_top = h * 3 // 4
        search_bot = h

        imgDia[0:search_top, 0:w] = 0
        imgDia[search_bot:h, 0:w] = 0
        # cv2.imshow('imagemask', imgDia)
        image1[0:search_top, 0:w1] = 0
        image1[search_bot:h, 0:w1] = 0

        image2[0:search_top, 0:w2] = 0
        image2[search_bot:h, 0:w2] = 0
        M = cv2.moments(imgDia)
        M1 = cv2.moments(image1)
        M2 = cv2.moments(image2)
        cx1 = 0
        cy1 = 0
        cy2 = 0
        cx2 = 0
        cx3 = 0
        cy3 = 0
        # cx22 = 0
        if M['m00'] > 0:
            cx1 = int((M['m10'] / M['m00']))
            cy1 = int(M['m01'] / M['m00'])
        if M1['m00'] > 0:
            cx2 = int((M1['m10'] / M1['m00']))
            cy2 = int(M1['m01'] / M1['m00'])
        if M2['m00'] > 0:
            cx3 = int((M2['m10'] / M2['m00']))
            cy3 = int(M2['m01'] / M2['m00'])
        if cx2 == 0 or (cx2 > 200 and cx3 > 400):
            cx2 = -100
        if cx3 == 0 or (cx3 < 400 and cx2 < 200):
            cx3 = 750
        err = cx1 - (cx2 + cx3) // 2;
        # 速度控制
        twist = Twist()
        if self.light(image) == 1:
            twist.linear.x = 0
            print("red")
        else:
            if self.zebra_judge == 1 and self.zebra_cnt <= 10:
                twist.linear.x = 0.1
                twist.angular.z = 0
                self.zebra_cnt += 2
            if M1['m00'] == 0:
                twist.linear.x = 0.3
                twist.angular.z = 0.1
                print("lost left")
            if M2['m00'] == 0:
                twist.linear.x = 0.3
                twist.angular.z = -0.1
                print("lost right")
            if M1['m00'] > 0 and M2['m00'] > 0:
                if self.zebra_cnt >= 6:
                    self.zebra_cnt -= 2
                    twist.linear.x = 0.1
                    twist.angular.z = 0
                    print("slow down")
                else:
                    twist.linear.x = 0.2
                    twist.angular.z = float(err) / 250
                print("xiuzheng")
        #     # 发布控制速度，实现对小车的控制
        self.cmd_vel_pub.publish(twist)

    # cv2.namedWindow('imagemask', cv2.WINDOW_NORMAL)
    # cv2.waitKey(1)
    # 红灯返回1 ，没有返回0
    def light(self, image):
        # 对红灯出现次数计数
        roiColor = image[220:350, 0:640]  # 640*480
        # 红灯处理
        hsv = cv2.cvtColor(roiColor, cv2.COLOR_BGR2HSV)
        lower_hsv_red = np.array([170, 100, 200])
        upper_hsv_red = np.array([180, 200, 255])
        mask_red = cv2.inRange(hsv, lowerb=lower_hsv_red, upperb=upper_hsv_red)
        # 绿灯处理
        lower_hsv_green = np.array([68, 150, 90])
        upper_hsv_green = np.array([80, 255, 120])
        mask_green = cv2.inRange(hsv, lowerb=lower_hsv_green, upperb=upper_hsv_green)
        red_color = np.max(mask_red)
        green_color = np.max(mask_green)
        if red_color == 255 and self.red_cnt <= 10:
            self.red_cnt += 1
        elif red_color != 255 and self.red_cnt > 0:
            self.red_cnt -= 1
        if self.red_cnt >= 7:
            return 1
        return 0
        # if green_color == 255:


# cv2.destroyWindow('imagemask')

rospy.init_node('line_follower')
follower = Follower()
rospy.spin()
