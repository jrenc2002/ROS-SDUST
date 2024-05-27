import cv2
import numpy as np


class LaneDetector:
    def __init__(self):
        self.zebra_judge = 0
        self.zebra_cnt = 0

    def detect_lane(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 0, 95])
        upper_hsv = np.array([180, 255, 255])
        imgDia = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
        h, w, d = image.shape
        image1 = imgDia[:, 0:int(w / 2)]
        image2 = imgDia[:, int(w / 2):w]

        h, w1 = image1.shape
        h, w2 = image2.shape

        search_top = h * 3 // 4
        search_bot = h

        imgDia[0:search_top, 0:w] = 0
        imgDia[search_bot:h, 0:w] = 0

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
        err = cx1 - (cx2 + cx3) // 2
        # print(err)

        # 根据err值进行状态检测输出
        return err
