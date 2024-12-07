import cv2 as cv
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from controller_manager_msgs.srv import SwitchController

class ServoCtl:
    def __init__(self):
        rospy.init_node('servo_ctl', anonymous=True)
        self.vel = TwistStamped()
        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(50)
        self.dashboard = np.zeros((480, 640, 3), np.uint8)
        # 定义一个字典映射按键到速度调整函数
        twist_vel = 0.2
        angular_vel = 0.7
        self.key_mappings = {
            'q': lambda: exit(),
            'w': lambda: setattr(self.vel.twist.linear, 'x', twist_vel),
            's': lambda: setattr(self.vel.twist.linear, 'x', -twist_vel),
            'a': lambda: setattr(self.vel.twist.linear, 'y', twist_vel),
            'd': lambda: setattr(self.vel.twist.linear, 'y', -twist_vel),
            ' ': lambda: setattr(self.vel.twist.linear, 'z', twist_vel),
            'c': lambda: setattr(self.vel.twist.linear, 'z', -twist_vel),
            'y': lambda: setattr(self.vel.twist.angular, 'x', angular_vel),
            'h': lambda: setattr(self.vel.twist.angular, 'x', -angular_vel),
            'u': lambda: setattr(self.vel.twist.angular, 'y', angular_vel),
            'j': lambda: setattr(self.vel.twist.angular, 'y', -angular_vel),
            'i': lambda: setattr(self.vel.twist.angular, 'z', angular_vel),
            'k': lambda: setattr(self.vel.twist.angular, 'z', -angular_vel),
            'r': lambda: self.controller_switch('servo'),
            't': lambda: self.controller_switch('position'),
            # 如果需要更多按键操作，请继续添加
        }

    
    def controller_switch(self, mode):
        rospy.wait_for_service('/controller_manager/switch_controller')
        switch_controller_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        if(mode =="position"):
            start_controllers = ['arm_controller']
            stop_controllers = ['joint_group_position_controller']
        elif(mode =="servo"):
            start_controllers = ['joint_group_position_controller']
            stop_controllers = ['arm_controller']
        else:
            rospy.logwarn("Invalid mode.")
            return

        strictness = 0
        start_asap = False
        timeout = 0.0
        response = switch_controller_service(start_controllers, stop_controllers, strictness, start_asap, timeout)
        if response.ok:
            rospy.loginfo("Successfully switched controllers.")
        else:
            rospy.logwarn("Failed to switch controllers.")



    def run(self):
        last_pressed_time = rospy.Time.now()
        while not rospy.is_shutdown():
            show_dashboard=self.dashboard.copy()
            cv.putText(show_dashboard, 'linear x: {:.2f}'.format(self.vel.twist.linear.x), (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'linear y: {:.2f}'.format(self.vel.twist.linear.y), (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'linear z: {:.2f}'.format(self.vel.twist.linear.z), (10, 90), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'angular x: {:.2f}'.format(self.vel.twist.angular.x), (10, 120), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'angular y: {:.2f}'.format(self.vel.twist.angular.y), (10, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'angular z: {:.2f}'.format(self.vel.twist.angular.z), (10, 180), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'press r to switch to servo mode', (10, 240), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'press to switch to position mode', (10, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv.putText(show_dashboard, 'press q to quit', (10, 210), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv.imshow('dashboard', show_dashboard)
            key = cv.waitKey(50)
            if key != -1:  # -1 表示没有按键被按下
                char_key = chr(key)  # 转换按键编码为字符
                if char_key in self.key_mappings:
                    key_action = self.key_mappings[char_key]
                    key_action()  # 执行对应的操作
            else:
                self.vel = TwistStamped()


            
            self.vel.header.stamp = rospy.Time.now()
            self.vel.header.frame_id = 'base_link'
            self.pub.publish(self.vel)

if __name__ == '__main__':
    ctl = ServoCtl()
    ctl.run()
    