import cv2
import numpy as np


class ZebraCrossingDetector:
    def __init__(self):
        self.zebra_judge = 0

    def detect_zebra_crossing(self, image):
        # 定义腐蚀和膨胀的核
        kernel_Ero = np.ones((3, 1), np.uint8)
        kernel_Dia = np.ones((5, 1), np.uint8)

        # 复制图像
        frame = cv2.resize(image, (640, 480))
        copy_img = frame.copy()

        # 获取图像的高度和宽度
        height, width = copy_img.shape[:2]

        # 设置感兴趣区域的左上角和右下角坐标
        roi_top_left = (width // 4, height//4)  # 左上角坐标为图像宽度的四分之一，高度为0
        roi_bottom_right = (width * 3 // 4, height*3//4)  # 右下角坐标为图像宽度的四分之三，高度为图像高度

        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.rectangle(mask, roi_top_left, roi_bottom_right, 255, -1)
        masked = cv2.bitwise_and(copy_img, copy_img, mask=mask)

        # 灰度化
        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)

        # 高斯滤波
        imgblur = cv2.GaussianBlur(gray, (5, 5), 10)

        # 阈值处理
        ret, thresh = cv2.threshold(imgblur, 200, 255, cv2.THRESH_BINARY)


        # 腐蚀
        img_Ero = cv2.erode(thresh, kernel_Ero, iterations=1)

        # 膨胀
        img_Dia = cv2.dilate(img_Ero, kernel_Dia, iterations=1)


        # 轮廓检测
        contours, _ = cv2.findContours(img_Dia, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = contours
        j = 0
        out = np.ones_like(copy_img) * 255  # 创建一个空的白色图像
        for contour in cnt:
            # 计算最小外接矩形
            rect = cv2.minAreaRect(contour)
            # 获取矩形的尺寸和角度信息
            rect_width, rect_height = rect[1]
            rect_angle = rect[2]

            # 筛选条件：长度大于100，高度大于15，且角度与水平线夹角不大于20度
            if rect_width > 20 and rect_height > 5 and rect_height<15 and abs(rect_angle) <= 20:
                # 绘制轮廓到图像上
                out = cv2.drawContours(copy_img, contour, -1, (0, 255, 0), 3)
                j += 1

        # cv2.imshow("out", out)
        # print("j:" + str(j))
        if 4 <= j <= 6:
            # print("slow down*****************************************")
            # 在这里添加减速操作，例如调用一个控制小车减速的函数
            self.go(0.05, 0)
            return 1
        return 0

    def go(self, linear_velocity, angular_velocity):
        # print("检测到了")
        # 在这里添加控制小车运动的代码，例如发布速度控制指令给小车
        pass


# 示例用法
if __name__ == '__main__':
    # 初始化斑马线检测器
    zebra_detector = ZebraCrossingDetector()

    # 加载图像，这里假设 image 是你的图像数据
    image = cv2.imread('../../image/2.png')
    cv2.imshow("image", image)

    # 进行斑马线检测
    zebra_detected = zebra_detector.detect_zebra_crossing(image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
