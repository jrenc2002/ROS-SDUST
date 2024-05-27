import cv2
import numpy as np

class ImageProcessor:
    def __init__(self):
        self.zebra_judge = 0

    def process_image(self, image):
        # 图像处理过程，包括斑马线检测等
        # 定义两个核（kernel_Ero用于腐蚀，kernel_Dia用于膨胀）
        kernel_Ero = np.ones((15, 3), np.uint8)
        kernel_Dia = np.ones((15, 3), np.uint8)

        # 将复制的图像裁剪为480*360
        copy_img = cv2.resize(image, (480, 360))
        imgGray = cv2.cvtColor(copy_img, cv2.COLOR_BGR2GRAY)
        imgBlur = cv2.GaussianBlur(imgGray, (5, 5), 0)
        ret, thresh = cv2.threshold(imgBlur, 200, 255, cv2.THRESH_BINARY)
        imgEro = cv2.erode(thresh, kernel_Ero, iterations=2)
        imgDia = cv2.dilate(imgEro, kernel_Dia, iterations=4)

        # 感兴趣区间
        h = imgDia.shape[0]
        w = imgDia.shape[1]
        poly = np.array([[(0, 360), (0, 240), (640, 240), (640, 360)]])
        mask = np.zeros_like(imgDia)
        cv2.fillPoly(mask, poly, 255)
        masked_image = cv2.bitwise_and(imgDia, mask)

        # 轮廓检测
        _, contouts, hie = cv2.findContours(masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = contouts

        xx = 0
        for i in cnt:
            x, y, w, h = cv2.boundingRect(i)
            if cv2.contourArea(i) > 1000 and cv2.contourArea(i) < 7000 and h > w:
                xx += 1
        if xx > 2:
            self.zebra_judge = 1
        else:
            self.zebra_judge = 0

        return self.zebra_judge
