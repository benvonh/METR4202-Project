#!/usr/bin/env python
import rospy
import math
import time
from sensor_msgs.msg import JointState


def add_angle(angles, increment):
    for i in range(4):
        angles[i] += increment
    return angles

def main():
    # setup
    pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=10)
    rospy.init_node("test2")
    rospy.sleep(3)
    rate = rospy.Rate(10)

    # Initial state
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    msg.position = [0, math.pi/2, 0, 0]
    msg.velocity = [1, 1, 1, 1]
    pub.publish(msg)
    rospy.loginfo("Initial state: 0")
    initialSleepTime = 5
    for i in range(initialSleepTime, 0, -1):
        rospy.loginfo(f"Starting in {i}...")
        time.sleep(1)

    incAngle = 0.1
    while not rospy.is_shutdown():
        if msg.position[0] > math.pi / 2:
            incAngle = -0.1
        if msg.position[0] < -math.pi / 2:
            incAngle = 0.1
        msg.position = add_angle(msg.position, incAngle)
        rospy.loginfo(f"Angle: {msg.position[0]}")
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

