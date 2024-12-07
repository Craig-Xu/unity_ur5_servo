#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.transformations as tft
from geometry_msgs.msg import TwistStamped


if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('listener', anonymous=True)

    vel = TwistStamped()
    pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)

    # 创建TF的监听器对象
    listener = tf.TransformListener()
    rate = rospy.Rate(50.0)  # 设置频率为10Hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('world', 'wrist_3_link', rospy.Time(0))


            # desired_matrix = [[0, 1, 0, 0],
            #                 [0, 0, -1, 0],
            #                 [-1, 0, 0, 0],
            #                 [0, 0, 0, 1]]

            desired_matrix = [[1, 0, 0, 0],
                            [0, 0, -1, 0],
                            [0, 1, 0, 0],
                            [0, 0, 0, 1]]
            
            desired_matrix = tft.inverse_matrix(desired_matrix)
            desired_rot = tft.quaternion_from_matrix(desired_matrix)
            delta_rot = tft.quaternion_multiply(desired_rot, tft.quaternion_inverse(rot))
            delta_matrix = tft.quaternion_matrix(delta_rot)
            angle, axis , _ = tft.rotation_from_matrix(delta_matrix)

            if angle<0:
                angle = -angle
                axis = -axis
            
            angular_p = 50.0
            angluar_velocity_limit = 10
            angluar_velocity = np.clip(angle*axis*angular_p, \
                                        -angluar_velocity_limit, angluar_velocity_limit)

            print('angluar_velocity:', angluar_velocity)

            vel.twist.angular.x = angluar_velocity[0]
            vel.twist.angular.y = angluar_velocity[1]
            vel.twist.angular.z = angluar_velocity[2]
            vel.header.stamp = rospy.Time.now()
            vel.header.frame_id = 'base_link'
            pub.publish(vel)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('waitting tf!')
            continue

        rate.sleep()
