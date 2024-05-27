import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv_bridge

from image_processing import ImageProcessor
from traffic_light_detection import TrafficLightDetector
from speed_control import SpeedController

def image_callback(msg):
    # 将ROS下图像格式转为opencv图像格式
    image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # 图像处理
    zebra_judge = image_processor.process_image(image)

    # 红绿灯检测
    red_cnt = light_detector.detect_light(image)

    # 速度控制
    twist = speed_controller.control_speed(zebra_judge, red_cnt, err)

    # 发布速度控制指令
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('line_follower')

    image_processor = ImageProcessor()
    light_detector = TrafficLightDetector()
    speed_controller = SpeedController()

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, image_callback)

    rospy.spin()
