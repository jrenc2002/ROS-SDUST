import time

import cv2
from image_processing import image_process
from traffic_light_detection import TrafficLightDetector
from zebra_crossing_detection import ZebraCrossingDetector
from lane_detection2 import LaneDetector
from car_control import CarController
from arrow_detection import ArrowDetector
from lane_detection import LineFollower


def main_control(car_controller, image):
    # 图像处理
    # masked_image = image_process(image)

    # 红绿灯检测
    light_detector = TrafficLightDetector()
    light_status = light_detector.detect_light(image)

    # 斑马线检测
    zebra_detector = ZebraCrossingDetector()
    zebra_status = zebra_detector.detect_zebra_crossing(image)

    # 加速箭头检测
    arrow_detector = ArrowDetector()
    arrow_status = arrow_detector.detect_arrow(image)

    # # 车道线检测
    # lane_detector = LaneDetector()
    # lane_err = lane_detector.detect_lane(image)

    # 车道线检测
    lineFollower=LineFollower()
    status,offset,slope=lineFollower.detect_lane(image)




    # 更新参数，进行小车控制
    car_controller.control(light_status, zebra_status,arrow_status,status,offset,slope)



# if __name__ == '__main__':
#     # 初始化车辆控制对象
#     car_controller = CarController()
#
#     # 初始化小车节点
#     image_processor = ImageProcessorROS()
#
#     # 获取图像
#     image=image_processor.image_callback()
#
#     # 进行控制
#     main_control(car_controller, image)
#
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()



# if __name__ == '__main__':
#     # 初始化车辆控制对象
#     car_controller = CarController()
#
#     # 加载图像
#     image = cv2.imread('../../image/test/19.jpg')
#
#     # 进行控制
#     main_control(car_controller, image)
#
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()


if __name__ == '__main__':
    # 初始化车辆控制对象
    car_controller = CarController()

    # 打开视频文件
    cap = cv2.VideoCapture('../../video/1.mp4')

    # 检查视频文件是否成功打开
    if not cap.isOpened():
        print("Error: Failed to open video file.")
        exit()

    # 记录上一次处理的时间
    last_process_time = time.time()

    while True:
        # 获取当前时间
        current_time = time.time()

        # 计算距离上次处理的时间间隔
        time_interval = current_time - last_process_time

        # 读取视频流的帧
        ret, frame = cap.read()

        # 检查帧是否成功读取
        if not ret:
            print("Error: Failed to read frame from video file.")
            break

        # 如果距离上次处理的时间超过0.5秒，则进行处理
        if time_interval >= 0.2:
            # 进行控制
            main_control(car_controller, frame)

            # 显示处理后的帧
            cv2.imshow("Processed Frame", frame)

            # 更新上一次处理的时间
            last_process_time = current_time

        # 检查是否按下了 ESC 键
        key = cv2.waitKey(1)
        if key == 27:  # ESC 键的 ASCII 码为 27
            break

    # 释放视频流对象
    cap.release()
    # 关闭所有 OpenCV 窗口
    cv2.destroyAllWindows()
