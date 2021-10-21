#!/usr/bin/env python
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from threading import Thread, Lock
import configuration as cf
import rospy
import numpy as np
import inverse_kinematics as ik


FILENAME = "~/catkin_ws/src/dynamixel_controller/src/config.json"


class Controller:
    def __init__(self):
        rospy.init_node("dynamixel_controller")
        self._jointPub = rospy.Publisher("/desired_joint_states", JointState)
        self._gripperPub = rospy.Publisher("/desired_gripper_state", Boolean)
        self._
        self._sub = rospy.Subscriber("/desired_configuration", Float64[])
        rospy.sleep(3)

        try:
            filename = rospy.get_params("config_filename")
        except:
            filename = FILENAME
        self._config = cf.ConfigJsonSerializer.deserialize(filename)

        self._msg = JointState()
        self._msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self._msg.velocity = np.ones(4)
        self._msg.effort = np.ones(4)

        rospy.loginfo("Moving to home configuration...")
        self._msg.position = np.zeros(4)
        self._pub.publish(self._msg)
        rospy.sleep(3)

        pos = [0.05, 0., -0.05]
        rot = [0, 0, 2]
        angles = ik.joint_angles(pos, rot, self._config.dimension)
        if angles is None:
            print("no solutions")
        else:
            self._msg = angles
            print(self._msg)
        print(self._msg)
        rospy.spin()

    def configuration_callback(self, data):

    def mainloop(configuration, gripper):
        


def main():
    controller = Controller()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
