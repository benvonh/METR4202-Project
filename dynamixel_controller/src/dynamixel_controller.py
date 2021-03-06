#!/usr/bin/env python
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from std_msgs.msg import Float64
from inverse_kinematics import joint_angles
import configuration as cf
import rospy
import numpy as np


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
        # Load config
        try:
            filename = rospy.get_params("config_filename")
        except:
            print("Using default file path")
            filename = FILENAME
        self._config = cf.ConfigJsonSerializer.deserialize(filename)

        # Node
        rospy.init_node("dynamixel_controller")
        self._joint_pub = rospy.Publisher("desired_joint_states", JointState,
                queue_size=10)
        self._code_pub = rospy.Publisher("configuration", Int8,
                queue_size=10)
        self._conf_sub = rospy.Subscriber("desired_configuration", Pose,
                self.set_configuration)
        self._joint_sub = rospy.Subscriber("joint_states", JointState,
                self.read_joint_state)
        rospy.sleep(3)
        rospy.loginfo("Initialised")

        # Messages
        self._joint = JointState()
        self._joint.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        self._joint.velocity = np.ones(4)
        self._joint.effort = np.zeros(4)
        self._code = Int8()
        self._code.data = 0

        # 0: stationary
        # 1: moving
        # 2: out of range                
        # 3: joint limit                 
        self._state = 0                
       
    def set_configuration(self, pose):
        pos = [pose.position.x, pose.position.y, pose.position.z]
        rot = [pose.orientation.x, pose.orientation.y, pose.orientation.z]
        angles = joint_angles(pos, rot, self._config.dimension)
        if type(angles) is str:
            if angles[0] == 'R':
                self._state = 2
            else:
                self._state = 3
        else:
            self._state = 1
        self._joint.position = angles

    def read_joint_state(self, joints):
        self._joint_angles = joints.position

    def mainloop(self):
        period = 2
        rate = rospy.Rate(10)
        count = 0
        bad_delay = 10
        while not rospy.is_shutdown():
            self._code_pub.publish(self._state)
            if self._state == 1: 
                rospy.loginfo(f"Moving joints to {self._joint.position}")
                self._joint_pub.publish(self._joint)
                if (np.abs(np.flip(np.array(self._joint.position)) - \
                        np.array(self._joint_angles)) < 0.087).all():
                    rospy.loginfo("Arrived at new joint states")
                    self._state = 0
            elif self._state > 1:
                if bad_delay == 10:
                    rospy.loginfo(f"Bad configuration (code {self._state})")
                    rospy.loginfo(self._joint.position)
                bad_delay -= 1
                if bad_delay == 0:
                    bad_delay = 10
                    self._state = 0
            elif count % 10 == 0:
                rospy.loginfo("No desired configuration")
            count += 1
            rate.sleep()


def main():
    controller = Controller()
    controller.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
