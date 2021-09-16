#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

def main():
    pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=10)
    rospy.init_node("test1")
    rate = rospy.Rate(5)
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    msg.position = [-2.0, -2.0, -2.0, -2.0]
    msg.velocity = [1, 1, 1, 1]
    pub.publish(msg)

    while not rospy.is_shutdown() and msg.position[0] < 3:
        rate.sleep()
        for i in range(4):
            msg.position[i] += 0.2
        pub.publish(msg)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

