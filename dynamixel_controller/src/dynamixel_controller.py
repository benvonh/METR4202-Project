#!/usr/bin/env python
from sensor_msgs.msg import JointState
import configuration as cf
import rospy
import numpy as np
import inverse_kinematics as ik


class Controller:
    def __init__(self):
        self._pub = rospy.Publisher("/desired_joint_states", JointState,
                queue_size=10)
        rospy.init_node("dynamixel_controller")
    
        rospy.sleep(3)

        self._config = cf.ConfigJsonSerializer.deserialize("config.json")
        #self._config = cf.ConfigJsonSerializer.deserialize(
        #        rospy.get_params("config_filename"))

        rospy.loginfo("Creating messages...")
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
            rospy.loginfo("no solutions")
        else:
            self._msg.position = angles
            self._pub.publish(self._msg)
        rospy.loginfo(self._msg.position)


    def mainloop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("temp")


def main():
    controller = Controller()
    #controller.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
