import cv2 as cv
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from controller_manager_msgs.srv import SwitchController
import tf
import tf.transformations as tft

class ServoCtl:
    def __init__(self):
        rospy.init_node('servo_ctl', anonymous=True)
        self.vel = TwistStamped()
        self.pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.rate = rospy.Rate(50)
        self.dashboard = np.zeros((480, 640, 3), np.uint8)
        self.listener = tf.TransformListener()
        self.gripper_command_pub = rospy.Publisher('/gripper_command', String, queue_size=10)
        self.gripper_distance = 0.0
        self.gripper_open_limit = 90.0
        self.gripper_close_limit = 0.0
        self.gripper_move_step = 1.0
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
            'o': lambda: self.gripper_ctl('open'),
            'i': lambda: self.gripper_ctl('close'),
            # 如果需要更多按键操作，请继续添加
        }

    def update_angluar_velocity(self, desired_matrix):
        self.listener.waitForTransform('/world', '/wrist_3_link', rospy.Time(0), rospy.Duration(4.0)) 
        (trans, rot) = self.listener.lookupTransform('world', 'wrist_3_link', rospy.Time(0))
        desired_matrix = tft.inverse_matrix(desired_matrix)
        desired_rot = tft.quaternion_from_matrix(desired_matrix)
        delta_rot = tft.quaternion_multiply(desired_rot, tft.quaternion_inverse(rot))
        delta_matrix = tft.quaternion_matrix(delta_rot)
        angle, axis , _ = tft.rotation_from_matrix(delta_matrix)

        angular_p = 50.0
        angluar_velocity_limit = 10.0
        angluar_velocity = np.clip(angle*axis*angular_p, \
                                    -angluar_velocity_limit, angluar_velocity_limit)
        self.vel.twist.angular.x = angluar_velocity[0]
        self.vel.twist.angular.y = angluar_velocity[1]
        self.vel.twist.angular.z = angluar_velocity[2]
        # print('angluar_velocity:', angluar_velocity)

    def gripper_ctl(self, cmd):
        if(cmd == 'open'):
            self.gripper_distance += self.gripper_move_step
        elif(cmd == 'close'):
            self.gripper_distance -= self.gripper_move_step
        else:
            rospy.logwarn("Invalid command.")
        self.gripper_distance = np.clip(self.gripper_distance, self.gripper_close_limit, self.gripper_open_limit)
        self.gripper_command_pub.publish(str(self.gripper_distance))

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
        key = -1
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
            tmp_key = cv.waitKey(10)
            if(key!=tmp_key):
                if(rospy.Time.now()-last_pressed_time>rospy.Duration(0.2)):
                    last_pressed_time = rospy.Time.now()
                    key = tmp_key

            if key != -1:  # -1 表示没有按键被按下
                char_key = chr(key)  # 转换按键编码为字符
                if char_key in self.key_mappings:
                    key_action = self.key_mappings[char_key]
                    key_action()  # 执行对应的操作
            else:
                self.vel.twist.linear.x = 0
                self.vel.twist.linear.y = 0
                self.vel.twist.linear.z = 0

            desired_matrix = [[-1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, -1, 0, 0],
                            [0, 0, 0, 1]]
            self.update_angluar_velocity(desired_matrix)
            
            self.vel.header.stamp = rospy.Time.now()
            self.vel.header.frame_id = 'base_link'
            self.pub.publish(self.vel)

if __name__ == '__main__':
    ctl = ServoCtl()
    ctl.run()
    