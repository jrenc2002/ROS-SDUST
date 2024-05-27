import cv2
import numpy as np

class TrafficLightDetector:
    def __init__(self):
        self.red_cnt = 0

    def detect_light(self, image):
        # 对红灯出现次数计数
        roiColor = image[220:350, 0:640]  # 640*480
        hsv = cv2.cvtColor(roiColor, cv2.COLOR_BGR2HSV)
        lower_hsv_red = np.array([170, 100, 200])
        upper_hsv_red = np.array([180, 200, 255])
        mask_red = cv2.inRange(hsv, lowerb=lower_hsv_red, upperb=upper_hsv_red)

        red_color = np.max(mask_red)
        if red_color == 255 and self.red_cnt <= 10:
            self.red_cnt += 1
        elif red_color != 255 and self.red_cnt > 0:
            self.red_cnt -= 1

        if self.red_cnt >= 7:
            return 1
        return 0
