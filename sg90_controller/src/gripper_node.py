#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import pigpio

def move_gripper(grip_state):
    """
    Function used to control gripper. When grip_state.data is true, the gripper is closed, and
    vice versa.
    :param grip_state: std_msg.msg.Bool object
    """
    if grip_state.data:
        grip_sg90.set_servo_pulsewidth(18, 1220) # Fully closed (add value to close looser)
        rospy.loginfo("The gripper has been closed")

    else:
        #grip_sg90.set_servo_pulsewidth(18, 1575) # Fully open (minus value to open less)
        grip_sg90.set_servo_pulsewidth(18, 1370) # 42mm open state.
        rospy.loginfo("The gripper has been opened")

    # publish to planner node when servo is finished moving? 

def grip_control():
    #initialise Node
    rospy.init_node('grip_control', anonymous = True)

    # subscribe to the topic that controls gripper
    rospy.Subscriber('/move_sg90', Bool, move_gripper)
    print("hi")
    rospy.spin()

if __name__ == '__main__':
    grip_sg90 = pigpio.pi()
    grip_sg90.set_mode(18, pigpio.OUTPUT)
    grip_control()
