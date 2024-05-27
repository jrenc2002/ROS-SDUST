stop_speed = 0
lower_speed = 10
normal_speed = 30
upper_speed = 50


class CarController:
    def __init__(self):
        # 初始化车速控制参数
        self.speed = normal_speed
        # 旋转角度
        self.steering_angle = 0
        # 是否是红灯
        self.is_red_light = False
        # 红灯检测次数,若处于非红灯状态，则当检测次数为2时，视为检测到红灯
        self.red_light_detection_count = 0
        # 非红灯检测次数，若处于红灯状态，当检测次数为3时，视为检测到非红灯
        self.non_red_light_detection_count = 0
        # 是否是斑马线
        self.is_zebra_crossing = False
        # 斑马线检测次数，若处于非斑马线状态，当检测次数为2时，视为检测到斑马线
        self.zebra_crossing_detection_count = 0
        # 未检测到斑马线的次数，若处于斑马线状态，当检测次数为2时，视为未检测到斑马线
        self.no_zebra_crossing_detection_count = 0
        # 是否开始加速
        self.is_start_accelerate = False
        # 是否结束加速
        self.is_end_accelerate = False
        # 是否处于加速中间状态，此时检测不到箭头
        self.is_during_accelerate = False
        # 加速箭头检测次数
        self.arrow_detection_count = 0
        # 是否经历过加速
        self.is_accelerated = False

    # 主控制函数
    def control(self, is_red_light, is_zebra_crossing, is_arrow, status, offset, slope):
        # 根据红绿灯状态和斑马线状态调整车速
        self.control_speed(is_red_light, is_zebra_crossing, is_arrow)
        # 根据车道线偏移误差调整转向角度
        self.control_steering(status, offset, slope)
        # 将参数传递给小车
        self.go()

    # 速度控制（主要用于红绿灯、斑马线以及加速线的检测控制）
    def control_speed(self, is_red_light, is_zebra_crossing, is_arrow):
        # 更新斑马线状态
        self.detect_zebra_crossing(is_zebra_crossing)
        # 更新加速线状态
        self.detect_arrow(is_arrow)
        # 更新红绿灯状态
        self.detect_light(is_red_light)

    # 方向控制（主要用于转弯）
    def control_steering(self, status, offset, slope):
        # T字型路口的旋转角度
        T_rotate = -60

        # 直行
        if status == 0:
            if abs(offset) < 20:
                self.steering_angle = 0
            else:
                self.steering_angle = 0.1 * offset
        # T字型路口
        elif status == 1:
            # 有其他逻辑控制，如红绿灯
            # 红灯向左转
            if self.is_red_light:
                self.steering_angle = T_rotate
            # 绿灯向右转
            else:
                self.steering_angle = T_rotate
        # 左转
        elif status == 2:
            self.steering_angle = -0.1 * offset
        # 右转
        elif status == 3:
            self.steering_angle = 0.1 * offset
        # 左转与直行分叉路口
        elif status == 4:
            # 有其他判断逻辑，比如红绿灯
            self.steering_angle = -0.1 * offset
        # 右转与直行分叉路口
        elif status == 5:
            # 有其他判断逻辑，比如红绿灯
            self.steering_angle = 0.1 * offset


    def go(self):
        speed = self.speed
        angle = self.steering_angle
        if self.is_red_light:
            print('当前处于红灯状态')
        else:
            if self.is_zebra_crossing:
                print('当前处于斑马线状态')
            else:
                if self.is_start_accelerate:
                    print('当前处于加速线状态')
                elif self.is_end_accelerate:
                    print('当前处于减速线状态')
                elif self.is_during_accelerate:
                    print('当前处于中间状态')
            print("speed:", speed, "angle:", angle)
        return speed, angle

    # 红绿灯状态判断（是否处于红绿灯状态）
    def detect_light(self, light_status):
        # 检测到是红灯，进行逻辑判断
        if light_status == 1:
            # 当前为非红灯状态
            if not self.is_red_light:
                # 红灯检测次数加1
                self.red_light_detection_count += 1
                # 如果达到次，视为红灯，并将非红灯检测次数清零
                if self.red_light_detection_count == 2:
                    self.is_red_light = True
                    self.non_red_light_detection_count = 0
            # 当前处于红灯状态，不管他
            else:
                print("当前处于红灯状态，就算检测到红灯也不管他")
        # 没有检测到红灯
        else:
            # 当前为红灯状态
            if self.is_red_light:
                # 非红灯检测次数加1
                self.non_red_light_detection_count += 1
                # 如果达到3次，视为非红灯，并将红灯检测次数清零
                if self.non_red_light_detection_count == 3:
                    self.is_red_light = False
                    self.red_light_detection_count = 0
            # 当前处于非红灯状态，不管他
            # else:
            #     print("当前处于非红灯状态，就算没有检测到红灯也不管他")

    # 斑马线状态判断（是否检测到斑马线）
    def detect_zebra_crossing(self, zebra_status):
        # 检测到是斑马线，进行逻辑判断
        if zebra_status == 1:
            # 当前为非斑马线状态
            if not self.is_zebra_crossing:
                # 斑马线检测次数加1
                self.zebra_crossing_detection_count += 1
                # 如果达到2次，视为斑马线，并将非斑马线检测次数清零,同时减速
                if self.zebra_crossing_detection_count == 2:
                    self.is_zebra_crossing = True
                    self.no_zebra_crossing_detection_count = 0
                    self.speed = lower_speed
            # 当前处于斑马线状态，不管他
            else:
                print("当前处于斑马线状态，就算检测到斑马线也不管他")
        # 没有检测到斑马线
        else:
            # 当前为斑马线状态
            if self.is_zebra_crossing:
                # 非斑马线检测次数加1
                self.no_zebra_crossing_detection_count += 1
                # 如果达到3次，视为非斑马线，并将斑马线检测次数清零,同时恢复到正常状态
                if self.no_zebra_crossing_detection_count == 3:
                    self.is_zebra_crossing = False
                    self.zebra_crossing_detection_count = 0
                    self.speed = normal_speed
            # 当前处于非斑马线状态，不管他
            # else:
            #     print("当前处于非斑马线状态，就算没有检测到斑马线也不管他")

    # 加速判断（是否检测到箭头）
    def detect_arrow(self, arrow_status):
        if self.is_accelerated:
            return
        else:
            if arrow_status == 1:
                # 如果没有开始加速也没有开始减速
                if not self.is_start_accelerate and not self.is_during_accelerate and not self.is_end_accelerate:
                    self.arrow_detection_count += 1
                    # 进入加速状态,更改速度参数
                    if self.arrow_detection_count == 4:
                        self.is_start_accelerate = True
                        self.arrow_detection_count = 0
                        self.speed = upper_speed

                # 如果处于中间态,则进行结束加速判断
                elif self.is_during_accelerate:
                    self.arrow_detection_count += 1
                    # 若处于结束加速状态，则将之前的状态重置，并设置速度恢复
                    if self.arrow_detection_count == 2:
                        self.is_end_accelerate = True
                        self.is_during_accelerate = False
                        self.arrow_detection_count = 0
                        self.speed = normal_speed

                # 如果处于加速状态，不管他
                # else:
                #     print("当前处于加速状态，就算检测到箭头也不管他")
            else:
                # 已经开始加速了，但没有处于中间态,此时进入中间态判断
                if self.is_start_accelerate and not self.is_during_accelerate:
                    self.arrow_detection_count += 1
                    # 进入中间态
                    if self.arrow_detection_count == 4:
                        self.is_during_accelerate = True
                        self.is_start_accelerate = False
                        self.arrow_detection_count = 0
                # 已经进入结束加速状态
                elif self.is_end_accelerate:
                    self.arrow_detection_count += 1
                    print("---------------", self.arrow_detection_count)
                    # 结束加速
                    if self.arrow_detection_count == 2 or self.arrow_detection_count > 2:
                        self.is_during_accelerate = False
                        self.is_end_accelerate = False
                        self.arrow_detection_count = 0
                        self.speed = normal_speed
                        self.is_accelerated = True
                else:
                    if self.arrow_detection_count != 0:
                        self.arrow_detection_count = 0
                #     print("当前处于非加速状态，就算没有检测到箭头也不管他")
