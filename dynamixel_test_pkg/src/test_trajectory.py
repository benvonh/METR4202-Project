#!/usr/bin/env python
import rospy
import math
import copy
from sensor_msgs.msg import JointState
import modern_robotics as mr
import numpy as np

gap1 = 0.026
gap2 = 0.037 + 0.0045 + 0.110

# points between screw axes
q1 = [0, 0, 0]
q2 = [0, 0, gap1]
q3 = [0, 0, gap1 + gap2]
q4 = [0.02, 0.01, gap1 + gap2 + 0.07]
# axis of rotation
w1 = np.array([0, 0, 1])
w2 = np.array([0, -1, 0])
w3 = np.array([0, -1, 0])
w4 = np.array([0, 0, 1])

Slist = np.array([
        np.concatenate((w1, np.cross(-w1, q1))),
        np.concatenate((w2, np.cross(-w2, q2))),
        np.concatenate((w3, np.cross(-w3, q3))),
        np.concatenate((w4, np.cross(-w4, q4)))
    ])

# home configuration
M = [
        [1, 0, 0, q4[0]],
        [0, 1, 0, q4[1]],
        [0, 0, 1, q4[2]],
        [0, 0, 0, 1]
    ]
eomg = 0.01
ev = 0.01

joint_max_angles = [
        (-math.pi, math.pi),
        (-math.pi/2, math.pi/2),
        (-5*math.pi/6, 5*math.pi/6),
        (-math.pi, math.pi)
    ]

def check_angles(angles):
    global joint_max_angles
    for i in range(4):
        range_ = joint_max_angles[i]
        if not range_[0] < angles[i] < range_[1]:
            return False
    return True

def get_desired(thetax, thetay, thetaz, x, y, z):
    global Slist, M, eomg, ev
    thetalist0 = [0 for _ in range(4)]
    rotx = np.array([
        [1, 0, 0],
        [0, math.cos(thetax), -math.sin(thetax)],
        [0, math.sin(thetax), math.cos(thetax)]
        ])
    roty = np.array([
        [math.cos(thetay), 0, math.sin(thetay)],
        [0, 1, 0],
        [-math.sin(thetay), 0, math.cos(thetay)]
        ])
    rotz = np.array([
        [math.cos(thetaz), -math.sin(thetaz), 0],
        [math.sin(thetaz), math.cos(thetaz), 0],
        [0, 0, 1]
        ])
    R = np.matmul(np.matmul(rotx, roty), rotz)
    p = np.array([x, y, z])
    T = mr.RpToTrans(R, p)
    sol = mr.IKinSpace(Slist.T, M, T, thetalist0, eomg, ev)
    print("Solution: ", sol)
    normalised = (sol[0] + np.pi) % (2 * np.pi) - np.pi
    print("Setting angles: ", normalised)
    return normalised

def main():
    global Slist, M, eomg, ev
    pub = rospy.Publisher("/desired_joint_states", JointState, queue_size=10)
    rospy.init_node("test_motion")
    rospy.sleep(3)
    rate = rospy.Rate(100)
    msg = JointState()
    msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
    msg.velocity = [0, 0, 0, 0]
    msg.effort = [0, 0, 0, 0]

    rospy.loginfo("Home configuration...")
    msg.velocity = [1, 1, 1, 1]
    msg.position = [0, 0, 0, 0]
    pub.publish(msg)
    rospy.sleep(3)
    msg.velocity = [0 for _ in range(4)]

    thetastart = [0 for _ in range(4)]
    stage = 1
    t = 0
    thetaend = None
    while not rospy.is_shutdown():
        if thetaend is None:
            if stage == 1:
                rospy.loginfo("Stage 1")
                thetaend = get_desired(0.2, -math.pi/2, math.pi, -0.2, -0.2, 0.2)
            elif stage == 2:
                rospy.loginfo("Stage 2")
                thetaend = get_desired(-0.2, -math.pi/2, 1.2, 0.2, 0.2, 0.2)
            elif stage == 3:
                rospy.loginfo("Stage 3")
                tehtaend = get_desired(math.pi/4, math.pi/2, -1, 0.2, -0.2, 0)
            else:
                rospy.loginfo("Finished...")
                break
            stage += 1
            t = 0

        s = 0.0892*t**2 - 0.01025*t**3
        if s > 1:
            rospy.loginfo("Arrived at destination!!!")
            theatstart = copy.deepcopy(thetaend)
            thetaend = None
            rospy.sleep(3)
            continue
        newAngles = thetastart + s * (thetaend - thetastart)
        if not check_angles(newAngles):
            rospy.loginfo("BAD ANGLES\n\t...closing")
            break
        msg.position = newAngles
        pub.publish(msg)
        rate.sleep()
        t += 0.01

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

