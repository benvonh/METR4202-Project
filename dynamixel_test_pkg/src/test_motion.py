#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import modern_robotics as mr
import numpy as np

# points between screw axes
q01 = [0, 0, 0]
q12 = [0, 0, 0.0325]
q23 = [0, 0, 0.125]
q34 = [0.02, 0.01, 0.1975]
# axis of rotation
w1 = np.array([0, 0, 1])
w2 = np.array([0, -1, 0])
w3 = np.array([0, -1, 0])
w4 = np.array([0, 0, 1])

Slist = np.array([
        np.concatenate((w1, np.cross(-w1, q01))),
        np.concatenate((w2, np.cross(-w2, q12))),
        np.concatenate((w3, np.cross(-w3, q23))),
        np.concatenate((w4, np.cross(-w4, q34)))
    ])

# home configuration
M = [
        [1, 0, 0, q34[0]],
        [0, 1, 0, q34[1]],
        [0, 0, 1, q34[2]],
        [0, 0, 0, 1]
    ]

# desired transformation
T = [
        [-1, 0, 0, 0.1],
        [0, 1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ]

thetalist0 = [0, 0, 0, 0]
eomg = 0.001
ev = 0.001

def main():
    global Slist, M, T, thetalist0, eomg, ev
    # ROS
    pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=10)
    rospy.init_node("test_motion")
    rate = rospy.Rate(0.2)
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    msg.velocity = [1, 1, 1, 1]
    msg.effort = [0.1, 0.1, 0.1, 0.1]

    initial = True
    while not rospy.is_shutdown():
        if not initial:
            # initial state
            rospy.loginfo("initial state")
            msg.position = [0, 0, 0, 0]
            pub.publish(msg)
            initial = True
        else:
            # desired state
            rospy.loginfo("calculating inverse kinematics")
            result = mr.IKinSpace(Slist.T, M, T, thetalist0, eomg, ev)
            rospy.loginfo("found real solution? " + "Yes" if result[1] else "No")
            rospy.loginfo("solution: " + str(result[0]))
            rospy.loginfo("desired state")
            msg.position = result[0].tolist()
            pub.publish(msg)
            initial = False
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

