#!/usr/bin/env python
import rospy
import tf

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('listener', anonymous=True)

    # 创建TF的监听器对象
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)  # 设置频率为10Hz
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('world', 'wrist_3_link', rospy.Time(0))
            
            rospy.loginfo("Rotation as Quaternion: %s", str(rot))

            mat = listener.fromTranslationRotation(trans, rot)
            
            # 打印姿态矩阵
            rospy.loginfo("Pose Matrix:\n%s", str(mat))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn('waitting tf!')
            continue

        rate.sleep()
