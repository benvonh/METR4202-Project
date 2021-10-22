#!/usr/bin/env python
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Bool
from threading import Thread, Lock
import numpy as np
import copy as cp
import tf2_ros
import rospy
import json


FILENAME = "/home/metr4202/catkin_ws/src/planner/src/preset.json"
COL_THRESH = 50


class Planner:
    def __init__(self):
        # Control
        presetFile = open(FILENAME, "r")
        self._preset = json.loads(presetFile.read())
        self._mutex = Lock()
        self._target = None
        self._colour = None

        # Node
        rospy.init_node("planner")
        self._conf_pub = rospy.Publisher("desired_configuration", Pose,
                queue_size=10)
        self._grip_pub = rospy.Publisher("desired_gripper_state", Bool,
                queue_size=10)
        rospy.Subscriber("/configuration", Int8, self.joint_code)
        rospy.Subscriber("/gripper_state", Bool, self.is_grip)
        rospy.Subscriber("/ximea_cam/image_raw", Image, self.image)
        rospy.Subscriber("/fiducial_transforms",
                FiducialTransformArray, self.transforms)
        rospy.Subscriber("/fiducial_vertices",
                FiducialArray, self.vertices)
        self._tfBuffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tfBuffer)
        rospy.sleep(3)
        rospy.loginfo("Initialised")

        # Message
        self._pose = Pose()
        self._grip = Bool()

        # Planning
        # WARNING: Do not access directly. Use corresponding methods instead to
        #          keep subscriber callbacks thread-safe.
        self.__joint_code = 0
        self.__is_grip = False
        self.__image = None
        self.__transforms = None
        self.__vertices = None
        self._stage = 0

    def joint_code(self, code: Int8):
        self._mutex.acquire()
        if code:
            self.__joint_code = code.data
        else:
            joint_code = self.__joint_code
        self._mutex.release()
        if not code:
            return joint_code

    def is_grip(self, grip: Bool):
        self._mutex.acquire()
        if grip:
            self.__is_grip = grip.data
        else:
            is_grip = self.__is_grip
        self._mutex.release()
        if not grip:
            return is_grip

    def image(self, data: Image):
        self._mutex.acquire()
        if data:
            self.__image = data
        else:
            image_data = cp.deepcopy(self.__image)
        self._mutex.release()
        if not data:
            return image_data

    def transforms(self, data: FiducialTransformArray):
        self._mutex.acquire()
        if data:
            print("expose you")
            self.__transforms = data.transforms
            print("self", self.__transforms)
        else:
            print("definitely exposed", self.__transforms)
            transforms_data = cp.deepcopy(self.__transforms)
            print(self.__transforms)

        self._mutex.release()
        if not data:
            return transforms_data

    def vertices(self, data: FiducialArray):
        self._mutex.acquire()
        if data:
            self.__vertices = data
        else:
            vertices_data = cp.deepcopy(self.__vertices)
        self._mutex.release()
        if not data:
            return vertices_data

    def set_pose_preset(self, preset):
        pos = preset["pos"]
        self._pose.position.x = pos['x']
        self._pose.position.y = pos['y']
        self._pose.position.z = pos['z']
        rot = preset["rot"]
        self._pose.orientation.x = rot['x']
        self._pose.orientation.y = rot['y']
        self._pose.orientation.z = rot['z']
        self._pose.orientation.w = rot['w']
    
    def set_pose_target(self, pos, rot):
        self._pose.position.x = pos[0]
        self._pose.position.y = pos[1]
        self._pose.position.z = pos[2]

    def set_colour(self):
        snap = self.images(None)
        array = self.vertices(None)
        vert = next((fid for fid in array if \
                fid.fiducial_id != self._target.fiducial_id), None)
        if vert is None:
            rospy.logwarn("Target fiducial not found")
            return
        index = (snap.height * (round(vert.y0) - 3) + round(vert.x0) - 3) * 3
        BGR = snap.data[index:index+3]
        if BGR[0] - BGR[1] > COL_THRESH and BGR[0] - BGR[2] > COL_THRESH:
            self._colour = "blue"
        elif BGR[1] - BGR[0] > COL_THRESH and BGR[1] - BGR[2] > COL_THRESH:
            self._colour = "green"
        elif BGR[2] - BGR[0] > COL_THRESH and BGR[2] - BGR[1] > COL_THRESH:
            self._colour = "red"
        elif BGR[1] - BGR[0] > COL_THRESH and BGR[2] - BGR[0] > COL_THRESH \
                and BGR[1] - BGR[2] < COL_THRESH:
            self._colour = "yellow"
        else:
            rospy.logwarn("Target colour could not be determined")
            return
        rospy.loginfo("Target colour is " + self._colour)

    def wait_move(self):
        while self.joint_code(None) == 1 and not rospy.is_shutdown():
            rospy.sleep(1)
        return self.joint_code(None)

    def wait_grip(self, set_grip):
        return
#        while self.is_grip(None) != set_grip and not rospy.is_shutdown():
#            rospy.sleep(1)

    def stage_1(self):
        rospy.loginfo("Moving to home configuration...")
        self.set_pose_preset(self._preset["home"])
        self._conf_pub.publish(self._pose)
        if self.wait_move() > 1:
            rospy.logwarn("Move unsuccessful!")
            rospy.logfatal("Please check home configuration preset")
            rospy.signal_shutdown("Unable to move to home configuration")
        self._stage = 2

    def stage_2(self):
        rospy.loginfo("Looking for fiducial marker...")
        rate = rospy.Rate(2)
        target_confirm = False
        while not target_confirm and not rospy.is_shutdown():
            scan = self.transforms(None)
            #scan = self.__transforms.transforms
            if len(scan) > 0:
                if self._target is None:
                    self._target = scan[0]
                elif self._target.fiducial_id == scan[0].fiducial_id:
                    target_reach = np.linalg.norm(
                            [
                                self._target.transform.translation.x,
                                self._target.transform.translation.y,
                                self._target.transform.translation.z
                            ])
                    scan_reach = np.linalg.norm(
                            [
                                scan[0].transform.translation.x,
                                scan[1].transform.translation.y,
                                scan[2].transform.translation.z
                            ])
                    shift = abs((target_reach - scan_reach) / target_reach)
                    if shift < 0.01:
                        rospy.loginfo("Fiducial marker found!")
                        self.set_colour()
                        target_confirm = True
                else:
                    target = None
            else:
                target = None
            rate.sleep()
        self._stage = 3

    def stage_3(self):
        rospy.loginfo("Moving to fiducial marker...")
        pos, rot = self._tfBuffer.lookupTransform(
                "fiducial_0",
                f"fiducial_{self._target.fiducial_id}",
                rospy.Time())
        pos[1] -= 0.055
        self.set_pose_target(pos, rot)
        self._conf_pub.publish(self._pose)
        if self.wait_move() > 1:
            rospy.logwarn("Move unsuccessful!")
            self._stage = 0
            return
        rospy.loginfo("Grabbing...")
        self._grip_pub.publish(True)
        self.wait_grip(True)
        self._stage = 4

    def stage_4(self):
        rospy.loginfo("Moving to drop-off zone...")
        dropzone = None
        for zone, spec in self._preset.items():
            if spec["colours"] == self._colour:
                dropzone = zone
                break
        if dropzone is None:
            rospy.logfatal("No colours match drop zones in preset.json")
            rospy.signal_shutdown("Bad preset file: colour not found")
            return
        self.set_pose_preset(self._preset[zone])
        self._conf_pub.publish(self._pose)
        if self.wait_move() > 1:
            rospy.logwarn("Move unsuccessful!")
            self._stage = 0
            return
        rospy.loginfo("Releasing...")
        self._grip_pub.publish(False)
        self.wait_grip(False)
        self._stage = 0

    def mainloop(self):
        stage = {
            1: self.stage_1,
            2: self.stage_2,
            3: self.stage_3,
            4: self.stage_4
        }
        self._stage = 1
        while not rospy.is_shutdown():
            exec_stage = stage.get(self._stage, None)
            if exec_stage is None:
                rospy.logfatal("Bad programming :(")
                break
            rospy.loginfo("==================================================")
            rospy.loginfo(f"Stage {self._stage}")
            rospy.loginfo("--------------------------------------------------")
            exec_stage()
            rospy.loginfo("__________________________________________________")


def main():
    node = Planner()
    node.mainloop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
