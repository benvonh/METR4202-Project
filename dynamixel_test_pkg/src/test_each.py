#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import JointState

def main():
    # Node
    pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=1)
    rospy.init_node("test_each")

    # Message
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    msg.position = [0, 0, 0, 0]
    msg.velocity = [1, 1, 1, 1]
    
    rospy.loginfo("Setting to 0 radians...\n")
    pub.publish(msg)
    time.sleep(4)

    i = 0
    while not rospy.is_shutdown():
        rospy.loginfo(f"Sweeping joint {i}...")

        rospy.loginfo("moving to 3 radians")
        msg.position[i] = 3
        time.sleep(4)

        rospy.loginfo("moving to -3 radians")
        msg.position[i] = -3
        time.sleep(7)

        rospy.loginfo("\n")
        time.sleep(1)
        msg.position = [0, 0, 0, 0]
        i += 1

    rospy.loginfo("Sweeping complete")


if __name__ == "__main__":
    try:
        rospy.loginfo("Ctrl+c now. Test does not work and time locks in main.")
        main()
    except rospy.ROSInterruptException:
        pass

