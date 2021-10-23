#!/usr/bin/env python
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from std_msgs.msg import Float64
import configuration as cf
import rospy
import numpy as np
import inverse_kinematics as ik


FILENAME = "/home/metr4202/catkin_ws/src/dynamixel_controller/src/config.json"


class Controller:
    """
    An intermediary node between the dynamixel interface and the desired pose.
    Subscribes to the desired_configuration topic for a pose and uses
    analytical inverse kinematics to publish to the desired_joint_states
    topic. Also publishes to the configuration topic with an integer code
    describing its state of motion or error.
    """

    def __init__(self):
        try:
            filename = rospy.get_params("config_filename")
        except:
            filename = FILENAME
        self._config = cf.ConfigJsonSerializer.deserialize(filename)

        rospy.init_node("dynamixel_controller")
        self._joint_pub = rospy.Publisher("desired_joint_states", JointState,
                queue_size=10)
        self._code_pub = rospy.Publisher("configuration", Int8,
                queue_size=10)
        self._sub = rospy.Subscriber("desired_configuration", Pose,
                self.set_configuration)
        self._rate = rospy.Rate(10)
        rospy.sleep(3)


        self._joint = JointState()
        self._joint.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self._joint.velocity = np.ones(4)
        self._joint.effort = np.ones(4)
        
        self._code = Int8()
        self._code.data = 0
       
    def set(self, data):
        pos = [data.position.x, data.position.y, data.position.z]
        rot = [0, 0, 0]
        angles = ik.joint_angles(pos, rot, self._config.dimension)
        if angles is None:
            self._temp = 2
            rospy.sleep(10)
            print("no solutions")
        else:
            self._temp = 1
            self._msg.position = angles
            print(self._msg)
            self._jointPub.publish(self._msg)
            rospy.sleep(5)
            self._temp = 0
 
    def main(self):
        while not rospy.is_shutdown():
            self._aa.data = self._temp
            self._pub.publish(self._aa)
            self._rate.sleep()


def main():
    controller = Controller()
    controller.main()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
