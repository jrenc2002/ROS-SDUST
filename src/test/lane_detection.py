import cv2
import numpy as np


class LineFollower:
    def __init__(self):
        # 车辆处于的状态
        # 直行为0，T字型路口（起始点）为1，左转路口为2，右转路口为3，
        # 左转与直行分叉路口为4，右转与直行分叉路口为5，十字路口为6
        self.status = 0
        # 偏移量，用于直行时偏移的判断；若处于非直行状态，只需根据状态进行对应操作
        self.offset = 0
        # 左转和右转时旋转量（这里使用斜率代替）
        self.slope = 90

    def detect_lane(self, image):
        # 获取最佳车道线
        top_lane, left_lane, right_lane = get_best_lane(image)
        print(top_lane, left_lane, right_lane)
        # 进行分类
        # T字型路口
        if top_lane is not None and left_lane is None and right_lane is None:
            self.status = 1
            print("当前处于T字型路口")
        # 左转路口
        elif top_lane is not None and left_lane is None and right_lane is not None:
            self.status = 2
            print("当前处于左转路口")
            self.slope = (right_lane[0][1] - right_lane[0][3]) / (right_lane[0][0] - right_lane[0][2])

        # 右转路口
        elif top_lane is not None and left_lane is not None and right_lane is None:
            self.status = 3
            print("当前处于右转路口")
            self.slope = (left_lane[0][1] - left_lane[0][3]) / (left_lane[0][0] - left_lane[0][2])

        # 左转与直行分叉路口
        elif top_lane is None and left_lane is None and right_lane is not None:
            self.status = 4
            print("当前处于左转与直行分叉路口")
            self.slope = (right_lane[0][1] - right_lane[0][3]) / (right_lane[0][0] - right_lane[0][2])

        # 右转与直行分叉路口
        elif top_lane is None and left_lane is not None and right_lane is None:
            self.status = 5
            print("当前处于右转与直行分叉路口")
            self.slope = (left_lane[0][1] - left_lane[0][3]) / (left_lane[0][0] - left_lane[0][2])

        # 十字路口
        # elif top_lane is None and left_lane is None and right_lane is None:
        #     self.status = 6
        #     print("当前处于十字路口")
        # 直行，计算偏移量
        else:
            self.status = 0
            if left_lane is None or right_lane is None:
                self.offset = 0
                print("当前处于直行")
            else:
                x, y = find_lines_intersection(left_lane, right_lane, image.shape)
                if x!=None:
                    self.offset = x - image.shape[1] / 2
                    print("当前处于直行,偏移量为:", self.offset)

        return self.status, self.offset,self.slope


def get_best_lane(image):
    # 获取3个处理后的感兴趣区域
    roi_top, roi_left, roi_right = region_of_interest(image)
    # 转为灰度图
    roi_top_gray = cv2.cvtColor(roi_top, cv2.COLOR_BGR2GRAY)
    roi_left_gray = cv2.cvtColor(roi_left, cv2.COLOR_BGR2GRAY)
    roi_right_gray = cv2.cvtColor(roi_right, cv2.COLOR_BGR2GRAY)
    cv2.imshow("roi_top_gray", roi_top_gray)
    cv2.imshow("roi_left_gray", roi_left_gray)
    cv2.imshow("roi_right_gray", roi_right_gray)

    # 获取最佳车道线
    roi_top_lane = get_best_line(get_lines(roi_top_gray, "top"), (image.shape[1] / 2, image.shape[0] / 2))
    roi_left_lane = get_best_line(get_lines(roi_left_gray, "left"), (image.shape[1] / 2, image.shape[0] / 2))
    roi_right_lane = get_best_line(get_lines(roi_right_gray, "right"), (image.shape[1] / 2, image.shape[0] / 2))

    return roi_top_lane, roi_left_lane, roi_right_lane


def get_lines(image_gray, region, min_length=30, max_length=float('inf')):
    # 使用Canny边缘检测
    edges = cv2.Canny(image_gray, 50, 150)

    # 使用霍夫线变换检测直线
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=15, minLineLength=15, maxLineGap=50)


    desLines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            # 计算线段的长度
            length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

            if x1 != x2:
                slope = (y2 - y1) / (x2 - x1)
                # 计算线段的角度
                # angle = np.arctan2(y2 - y1, x2 - x1) * 180.0 / np.pi

                # 根据不同区域进行筛选
                if region == "top":
                    # 顶部车道线筛选条件：接近水平线
                    if abs(slope) <= 0.15 and min_length <= length <= max_length:
                        desLines.append(line)
                elif region == "left":
                    # 左侧车道线筛选条件：角度在 -60 度至 -30 度之间
                    if slope < -0.15 and min_length <= length <= max_length:
                        desLines.append(line)
                elif region == "right":
                    # 右侧车道线筛选条件：角度在 30 度至 60 度之间
                    if slope > 0.15 and min_length <= length <= max_length:
                        desLines.append(line)

    return desLines


def get_best_line(lines, center_point):
    if len(lines) == 0:
        return None

    best_line = None
    best_distance = float('inf')

    # 根据线段长度和距离中心点的距离排序
    for line in lines:
        x1, y1, x2, y2 = line[0]  # 解包线段坐标
        # 计算线段长度
        length = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        # 计算线段中心点坐标
        line_center = ((x1 + x2) / 2, (y1 + y2) / 2)
        # 计算中心点到线段的距离
        distance = np.sqrt((center_point[0] - line_center[0]) ** 2 + (center_point[1] - line_center[1]) ** 2)

        # 根据长度和距离中心点的距离计算加权得分
        score = 0.6 * length + 0.4 * distance

        # 更新最佳线段
        if score < best_distance:
            best_distance = score
            best_line = line

    return best_line


"""
    计算左侧车道线与右侧车道线的交点
"""


def find_lines_intersection(left_lines, right_lines, img_shape):
    # 计算左侧和右侧直线的交点
    for left_line in left_lines:
        for right_line in right_lines:
            x1, y1, x2, y2 = left_line
            x3, y3, x4, y4 = right_line
            # 计算两条直线的斜率
            m1 = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else np.inf
            m2 = (y4 - y3) / (x4 - x3) if (x4 - x3) != 0 else np.inf

            # 如果其中一条直线是垂直线
            if np.isinf(m1):
                # 计算垂直线的x坐标
                x = x1
                # 计算水平线的y坐标
                y = m2 * (x - x3) + y3
            elif np.isinf(m2):
                # 计算垂直线的x坐标
                x = x3
                # 计算水平线的y坐标
                y = m1 * (x - x1) + y1
            else:
                # 如果两条直线都不是垂直线，则计算交点
                x = (m1 * x1 - m2 * x3 + y3 - y1) / (m1 - m2)
                y = m1 * (x - x1) + y1

            # 确保交点在图像范围内
            if 0 <= x <= img_shape[1]:
                return x, y

    return None, None


"""
    获取感兴趣区域
"""


def region_of_interest(img):
    # 如果输入图像为BGR格式，转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) > 2 and img.shape[2] == 3 else img
    # 对灰度图进行二值化处理
    _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    # 二值化后的图像与原始图像进行与操作，保留车道线区域
    masked_image = cv2.bitwise_and(img, img, mask=binary)
    cv2.imshow("masked_image", masked_image)

    # 定义腐蚀和膨胀的结构元素（核）
    kernel = np.ones((1, 3), np.uint8)
    # kernel1 = np.ones((3, 1), np.uint8)
    # 对图像进行膨胀
    masked_image = cv2.dilate(masked_image, kernel, iterations=4)
    # masked_image = cv2.erode(masked_image, kernel1, iterations=3)


    # 获取图像的高度和宽度
    height, width = masked_image.shape[:2]

    # 获取三个感兴趣区域的顶点坐标
    roi_top_vertices, roi_left_vertices, roi_right_vertices = get_roi_vertices(img)

    # 创建与输入图像大小相同的全零掩膜
    mask1 = np.zeros_like(masked_image)
    mask2 = np.zeros_like(masked_image)
    mask3 = np.zeros_like(masked_image)

    # 在掩膜上填充矩形，只保留各自的感兴趣区域
    cv2.fillPoly(mask1, roi_top_vertices, (255,) * masked_image.shape[2])
    cv2.fillPoly(mask2, roi_left_vertices, (255,) * masked_image.shape[2])
    cv2.fillPoly(mask3, roi_right_vertices, (255,) * masked_image.shape[2])

    # 进行与操作，得到各自的感兴趣区域
    roi_top = cv2.bitwise_and(masked_image, mask1)
    roi_left = cv2.bitwise_and(masked_image, mask2)
    roi_right = cv2.bitwise_and(masked_image, mask3)

    # 闭运算（填充边缘之间的空洞）
    roi_top = cv2.morphologyEx(roi_top, cv2.MORPH_CLOSE, kernel)
    roi_left = cv2.morphologyEx(roi_left, cv2.MORPH_CLOSE, kernel)
    roi_right = cv2.morphologyEx(roi_right, cv2.MORPH_CLOSE, kernel)

    return roi_top, roi_left, roi_right


def get_roi_vertices(img):
    # 获取图像的高度和宽度
    height, width = img.shape[:2]
    roi_top_vertices = np.array(
        [[(width // 3, height * 4 // 5), (width * 2 // 3, height * 4 // 5), (width * 2 // 3, height),
          (width // 3, height)]], dtype=np.int32)
    roi_left_vertices = np.array(
        [[(0, height // 2), (width // 4, height // 2), (width // 4, height), (0, height)]],
        dtype=np.int32)
    roi_right_vertices = np.array(
        [[(width*3 // 4, height // 2), (width, height // 2), (width, height), (width*3 // 4, height)]],
        dtype=np.int32)
    return roi_top_vertices, roi_left_vertices, roi_right_vertices


if __name__ == '__main__':
    # 加载图像
    image = cv2.imread('../../image/load/6.png')
    lineFollower = LineFollower()

    # 进行图像处理
    lineFollower.detect_lane(image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
