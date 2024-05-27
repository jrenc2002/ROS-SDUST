import cv2
import numpy as np

class TrafficLightDetector:


    def detect_light(self, image):
        # 获取图像的高度（h）、宽度（w）和深度（d）
        h, w, d = image.shape

        # 仅处理图像中间一部分区域
        roi_top = int(h / 3)
        roi_bottom = int(2 * h / 3)
        roi_left = int(w / 3)
        roi_right = int(2 * w / 3)
        roi = image[roi_top:roi_bottom, roi_left:roi_right]

        # 将图像从BGR颜色空间转换为HSV颜色空间
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 定义红色的HSV阈值范围
        lower_red1 = np.array([0, 120, 220])
        upper_red1 = np.array([4, 255, 255])
        lower_red2 = np.array([165, 170, 180])
        upper_red2 = np.array([179, 255, 255])
        # 定义绿色的HSV阈值范围
        lower_green = np.array([40, 50, 50])
        upper_green = np.array([90, 255, 255])

        # 创建红色和绿色掩膜
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # 形态学操作：开运算去除噪点
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

        # 计算红色和绿色掩膜的像素总数
        red_pixel_count = np.sum(mask_red)
        green_pixel_count = np.sum(mask_green)

        # 设置阈值来判断红灯和绿灯
        red_threshold = 10000  # 修改红色阈值
        green_threshold = 10000  # 修改绿色阈值

        if red_pixel_count > red_threshold:
            signal = 1
        elif green_pixel_count > green_threshold:
            signal = 0
        else:
            signal = 0
        return signal

