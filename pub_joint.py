# rostopic pub /ur5e/joint_command sensor_msgs/JointState "header:
#   seq: 0
#   stamp: {secs: 0, nsecs: 0}
#   frame_id: ''
# name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
# position: [0.3, 0, 0, 0, 0, 0]
# velocity: [0]
# effort: [0]"
import rospy
from sensor_msgs.msg import JointState
import time
class JointCommand:
    def __init__(self):
        self.pub = rospy.Publisher('/ur5e/joint_command', JointState, queue_size=10)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback)
        self.rate = rospy.Rate(10)  # 10Hz

    def callback(self, data):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['shoulder_pan_joint' , 'shoulder_lift_joint','elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        tmp = data.position
        # 转为列表
        tmp = list(tmp)
        tp = tmp[0]
        tmp[0] = tmp[2]
        tmp[2] = tp
        joint_state.position = tmp
        joint_state.velocity = data.velocity
        self.pub.publish(joint_state)
    def run(self):
        while not rospy.is_shutdown():
            print("hello")
            time.sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('joint_command_publisher')
    joint_command = JointCommand()
    joint_command.run()
