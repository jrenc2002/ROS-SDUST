from geometry_msgs.msg import Twist

class SpeedController:
    def __init__(self):
        self.zebra_cnt = 0

    def control_speed(self, zebra_judge, red_cnt, err):
        twist = Twist()

        if red_cnt >= 7:
            print("红灯停止")
            twist.linear.x = 0
        else:
            if zebra_judge == 1 and self.zebra_cnt <= 10:
                print("减速通过斑马线")
                twist.linear.x = 0.1
                twist.angular.z = 0
                self.zebra_cnt += 2
            else:
                if err == 0:
                    print("直行")
                    twist.linear.x = 0.2
                else:
                    print("根据偏差调整方向")
                    twist.linear.x = 0.2
                    twist.angular.z = float(err) / 250

        return twist
