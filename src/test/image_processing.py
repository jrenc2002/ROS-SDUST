import cv2
import numpy as np


def image_process(image):
    # 定义两个核（kernel_Ero用于腐蚀，kernel_Dia用于膨胀）
    kernel_Ero = np.ones((15, 3), np.uint8)
    kernel_Dia = np.ones((15, 3), np.uint8)

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

    return masked_image
