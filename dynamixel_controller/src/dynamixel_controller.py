#!/usr/bin/env python
import sys

# Import ROS components
from sensor_msgs.msg import JointState
import rospy

# Import inverse kinematics
import modern_robotics as mr
import numpy as np


class Controller:
    def __init__(self):
        rospy.loginfo("Initialising...")
        self._pub = rospy.Publisher("/desired_joint_states", JointState,
                queue_size=10, anonymous=True)
        self.rate = rospy.Rate(RATE_HZ)
        rospy.init_node("dynamixel_controller")
        rospy.sleep(3)

        rospy.loginfo("Creating messages...")
        self._msg = JointState()
        self._msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self._msg.velocity = np.ones(4)
        self._msg.effort = np.ones(4)

        rospy.loginfo("Press enter to move to home configuration.")
        self._msg.position = np.zeros(4)
        input()
        self._pub.publish(self._msg)
        rospy.sleep(3)

        rospy.loginfo("End of program")


    def mainloop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("temp")


def main():
    controller = Controller()
    controller.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
