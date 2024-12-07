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


    # 画姿态坐标系在图片上
    def draw_axes(self, img, R, t):
        Rt = np.hstack((R, t))
        Rt = np.vstack((Rt, [0, 0, 0, 1]))
        length = 0.1
        axis_length = length * 0.1
        x_axis = np.array([[0, 0, 0, 1], [length, 0, 0, 1]])
        y_axis = np.array([[0, 0, 0, 1], [0, length, 0, 1]])
        z_axis = np.array([[0, 0, 0, 1], [0, 0, length, 1]])
        x_axis = Rt @ x_axis
        y_axis = Rt @ y_axis


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
            key = cv.waitKey(5)
            if key != -1:
                last_pressed_time = rospy.Time.now()

            if key == ord('q'):
                break
            elif key == ord('w'):
                self.vel.twist.linear.x = 0.1
            elif key == ord('s'):
                self.vel.twist.linear.x = -0.1
            elif key == ord('a'):
                self.vel.twist.linear.y = 0.1
            elif key == ord('d'):
                self.vel.twist.linear.y = -0.1
            elif key == ord(' '):
                self.vel.twist.linear.z = 0.1
            elif key == ord('c'):
                self.vel.twist.linear.z = -0.1
            elif key == ord('y'):
                self.vel.twist.angular.x = 0.7
            elif key == ord('h'):
                self.vel.twist.angular.x = -0.7
            elif key == ord('u'):
                self.vel.twist.angular.y = 0.7
            elif key == ord('j'):
                self.vel.twist.angular.y = -0.7
            elif key == ord('i'):
                self.vel.twist.angular.z = 0.7
            elif key == ord('k'):
                self.vel.twist.angular.z = -0.7
            elif key == ord('r'):
                self.controller_switch('servo')
            elif key == ord('t'):
                self.controller_switch('position')
            else:
                # if(rospy.Time.now() - last_pressed_time).to_sec() > 0.1:
                self.vel = TwistStamped()
            
            self.vel.header.stamp = rospy.Time.now()
            self.vel.header.frame_id = 'base_link'
            self.pub.publish(self.vel)

if __name__ == '__main__':
    ctl = ServoCtl()
    ctl.run()
    