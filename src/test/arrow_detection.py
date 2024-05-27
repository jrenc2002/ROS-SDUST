import cv2
import numpy as np

class ArrowDetector:
    def __init__(self):
        pass

    def detect_arrow(self, image):
        # 转换颜色空间为 HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 创建白色掩码
        white_lower = np.array([0, 0, 200], dtype=np.uint8)
        white_upper = np.array([255, 20, 255], dtype=np.uint8)
        white_mask = cv2.inRange(hsv_image, white_lower, white_upper)

        # 创建感兴趣区域掩码
        roi_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        h, w = image.shape[:2]
        roi_mask[h * 3 // 5:h, w // 7:w * 6 // 7] = 255

        # 结合白色掩码和感兴趣区域掩码
        masked_image = cv2.bitwise_and(white_mask, white_mask, mask=roi_mask)

        # 形态学处理
        kernel = np.ones((3, 3), np.uint8)
        opened_image = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(opened_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        flag=False

        # 绘制轮廓
        for cnt in contours:
            # 计算轮廓周长和多边形逼近
            length = cv2.arcLength(cnt, True)
            epsilon = 0.03 * length
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # 根据条件判断轮廓形状
            if length > 230 and (len(approx) == 7 or len(approx) == 8):
                # 计算轮廓的面积和周长
                area = cv2.contourArea(approx)
                perimeter = cv2.arcLength(approx, True)
                # 计算图像的宽高比
                aspect_ratio = float(w) / h
                # 根据条件筛选合适的轮廓
                # and aspect_ratio >= 0.8 and aspect_ratio <= 1.2
                if area > 160 and perimeter > 30 :
                    x, y, w, h = cv2.boundingRect(approx)
                    flag=True
                    cv2.rectangle(image, (int(x), int(y)), (int(x + w), int(y + h)), (255, 0, 0), 2)  # 绘制矩形框

                    ellipse = cv2.fitEllipse(approx)
                    cv2.ellipse(image, ellipse, [255, 0, 0], 2)  # 拟合椭圆

                    # 计算轮廓的中心点坐标
                    M = cv2.moments(approx)
                    if M['m00'] != 0:
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                    else:
                        cx = cy = 0

                    # 获取中心点和目标点坐标
                    dstx, dsty = self.get_center_of_contour(cx, cy, approx)

                    # 绘制轮廓和中心点
                    cv2.drawContours(image, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(image, (cx, cy), 5, (0, 255, 255), -1)
                    cv2.circle(image, (dstx, dsty), 5, (255, 0, 255), -1)

                    # 测试
                    cv2.drawContours(opened_image, [approx], -1, (0, 255, 0), 2)
                    cv2.circle(opened_image, (cx, cy), 5, (0, 255, 255), -1)
                    cv2.circle(opened_image, (dstx, dsty), 5, (255, 0, 255), -1)
                    # cv2.imshow("test",opened_image)


                    # 根据中心点和目标点位置显示角度信息
                    if dsty - cy <= 0:
                        cv2.putText(image, "UP+Angle:" + str(int(ellipse[2])), (x + int(0.5 * w), y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 0, 255), 1, cv2.LINE_AA)
                    else:
                        cv2.putText(image, "DOWN+Angle:" + str(int(ellipse[2])), (x + int(0.5 * w), y), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5, (0, 0, 255), 1, cv2.LINE_AA)

        # 如果检测到了返回1，反之返回0
        if flag:
            return 1
        else:
            return 0


    def get_center_of_contour(self, cx, cy, approx):
        base_x = cx
        base_y = cy

        # 对每个轮廓中的点进行距离计算，并存储为 (point, distance) 的元组
        contours_with_distances = []

        for point in approx:
            distance = self.distance_to_base(point[0], cx, cy)
            contours_with_distances.append((point[0], distance))

        # 根据距离对轮廓进行排序
        sorted_contours = sorted(contours_with_distances, key=lambda x: x[1], reverse=True)

        # 三角形的三个顶点坐标
        p1 = sorted_contours[2][0]
        p2 = sorted_contours[3][0]
        p3 = sorted_contours[4][0]
        point1 = (p1[0], p1[1])
        point2 = (p2[0], p2[1])
        point3 = (p3[0], p3[1])

        # 计算重心坐标
        centroid = self.triangle_centroid(point1, point2, point3)
        dstx = int(centroid[0])
        dsty = int(centroid[1])

        return dstx, dsty

    def distance_to_base(self, point, base_x, base_y):
        x, y = point
        return np.sqrt((x - base_x) ** 2 + (y - base_y) ** 2)

    def triangle_centroid(self, point1, point2, point3):
        # 计算每个坐标轴上的平均值
        centroid_x = (point1[0] + point2[0] + point3[0]) / 3.0
        centroid_y = (point1[1] + point2[1] + point3[1]) / 3.0
        return (centroid_x, centroid_y)
# 测试
# if __name__ == "__main__":
#     detector = ArrowDetector()
#     detector.detect_arrow("../../image/arrow/6.png")
