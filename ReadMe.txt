自动驾驶小车项目说明
项目概述
该项目旨在开发一个自动驾驶小车系统，该系统能够检测红绿灯和斑马线，以实现智能驾驶功能。

项目架构
项目主要分为以下模块：

main.py: 主文件，负责整体流程控制，调用图像处理、红绿灯检测和斑马线检测模块。
image_processing.py: 图像处理模块，实现图像预处理功能，包括灰度转换、高斯滤波、阈值处理等。
traffic_light_detection.py: 红绿灯检测模块，用于检测红绿灯的状态，返回红绿灯的信号。
zebra_crossing_detection.py: 斑马线检测模块，用于检测斑马线，返回斑马线的状态。

各模块功能和具体实现
image_processing.py
image_process(image): 接收一张图像作为输入，对图像进行预处理，包括灰度转换、高斯滤波、阈值处理等，最终返回处理后的图像。
traffic_light_detection.py
TrafficLightDetector: 红绿灯检测类，包含红灯计数器和红绿灯检测方法。
detect_light(image): 接收一张图像作为输入，从图像中提取出红绿灯区域，然后利用颜色识别方法检测红灯状态，返回红灯状态（红灯为1，绿灯为0）。
zebra_crossing_detection.py
ZebraCrossingDetector: 斑马线检测类，包含斑马线检测方法。
detect_zebra_crossing(image): 接收一张图像作为输入，利用图像处理技术检测斑马线，返回斑马线状态（检测到斑马线为1，否则为0）。

使用方法
将待处理的图像放置在 image 文件夹中。
运行 main.py 文件即可启动自动驾驶小车系统，系统会检测红绿灯和斑马线，并输出相应的结果。