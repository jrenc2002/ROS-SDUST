import cv2
import numpy as np


class ObstacleDetector:
    def __init__(self):
        pass

    def detect_obstacle(self, image):
        # 转换颜色空间为灰度
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 应用高斯模糊以减少噪声
        blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)

        # 使用Canny边缘检测
        edged_image = cv2.Canny(blurred_image, 50, 150)

        # 查找轮廓
        contours, _ = cv2.findContours(edged_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        flag = False

        # 绘制轮廓
        for contour in contours:
            # 计算轮廓面积，忽略小轮廓
            area = cv2.contourArea(contour)
            if area > 500:
                flag = True
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # 绘制矩形框
                cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)  # 绘制轮廓

        # 如果检测到了返回1，反之返回0
        if flag:
            return 1
        else:
            return 0


# 测试
if __name__ == "__main__":
    detector = ObstacleDetector()
    cap = cv2.VideoCapture(0)  # 使用摄像头

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        result = detector.detect_obstacle(frame)
        cv2.imshow('Obstacle Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
