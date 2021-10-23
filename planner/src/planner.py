#!/usr/bin/env python
from fiducial_msgs.msg import FiducialTransformArray, FiducialArray
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, Bool
from threading import Thread, Lock
import color_detector as cd
import numpy as np
import copy as cp
import tf2_ros
import rospy
import json


FILENAME = "/home/metr4202/catkin_ws/src/planner/src/preset.json"
BASE_FIDUCIAL = 4


class Planner:
    """
    A planner node responsible for publishing the arm configuration and gripper
    state. Iterates through four stages for each luggage successfully placed
    in the drop-zone.
    
    @stage_1: Moves the arm to its home configuration.
    @stage_2: Awaits for a fiducial marker to appear and remain stationary.
              Records the colour of the target luggage.
    @stage_3: Moves the arm to grab the target luggage.
    @stage_4: Places the luggage at the correspondig drop-off zone matching
              the preset colour. Resets to @stage_1.
    """

    def __init__(self):
        """Initialises all components of the Planner object including the node.
        Loads the preset json file and sleeps 3 seconds before completing."""
        # Control
        presetFile = open(FILENAME, "r")
        self._preset = json.loads(presetFile.read())
        self._mutex = Lock()
        self._target = None

        # Colour detection
        current_dir = "/home/metr4202/catkin_ws/src/planner/src"
        try:
            file = open(current_dir + "/colors.config", 'r')
            i = 0
            for line in file:
                entries = line.split(' ')
                values = [int(x) for x in entries]
                if i == 0:
                    bgr_r = np.array([values[0], values[1], values[2]]).astype(np.uint8)
                if i == 1:
                    bgr_g = np.array([values[0], values[1], values[2]]).astype(np.uint8)
                if i == 2:
                    bgr_b = np.array([values[0], values[1], values[2]]).astype(np.uint8)
                if i == 3:
                    bgr_y = np.array([values[0], values[1], values[2]]).astype(np.uint8)
                i += 1
        except FileNotFoundError:
            print("Could not load color.config file")
            print("Setting to default values:")
            bgr_r = np.array([0, 0, 255]).astype(np.uint8)
            bgr_g = np.array([0, 255, 0]).astype(np.uint8)
            bgr_b = np.array([255, 0, 0]).astype(np.uint8)
            bgr_y = np.array([0, 255, 255]).astype(np.uint8)

        self._colour_detector = cd.ColorDetector(bgr_r, bgr_g, bgr_b, bgr_y)
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
            self.__transforms = data.transforms
        else:
            transforms_data = cp.deepcopy(self.__transforms)
        self._mutex.release()
        if not data:
            return transforms_data

    def vertices(self, data: FiducialArray):
        self._mutex.acquire()
        if data:
            self.__vertices = data.fiducials
        else:
            vertices_data = cp.deepcopy(self.__vertices)
        self._mutex.release()
        if not data:
            return vertices_data

    def set_pose_preset(self, preset):
        """Sets the pose to the preset."""
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
        """Sets the pose to the postion and rotation."""
        self._pose.position.x = pos.x
        self._pose.position.y = pos.y
        self._pose.position.z = pos.z

    def set_colour(self):
        """"""
        self_colour = 'red'
        rospy.loginfo("lol its red")
        return

        offset = 1
        while self._colour is None and not rospy.is_shutdown():
            snap = self.image(None)
            array = []
            while len(array) == 0 and not rospy.is_shutdown():
                array = self.vertices(None)
            vert = array[0]
            #vert = next((fid for fid in array if \
            #        fid.fiducial_id == self._target.fiducial_id), None)
            if vert is None:
                rospy.logfatal("Target fiducial not found")
                return
            x = min(vert.x0, min(vert.x1, min(vert.x2, vert.x3)))
            y = min(vert.y0, min(vert.y1, min(vert.y2, vert.y3)))
            rospy.loginfo(f"Sweeping pixel colours [{offset}]")
            x = 0
            y = 0
            while not rospy.is_shutdown():
                snap = self.image(None)
                index = int((snap.width * (round(x) - offset) + y - offset) * 3)
                BGR = np.array([int(bit) for bit in \
                            snap.data[index:index + 3]]).astype(np.uint8)
                rospy.loginfo(f"x: {x}, y: {y}")
                rospy.loginfo(f"BGR: {int(BGR[0])} {int(BGR[1])} {int(BGR[2])}")
                rospy.loginfo("Colour: " + self._colour_detector.detect_color(BGR))


                x += 1
                y += 1


                """
                array = []
                while len(array) == 0 and not rospy.is_shutdown():
                    array = self.vertices(None)
                vert = array[0]
                #vert = next((fid for fid in array if \
                #        fid.fiducial_id == self._target.fiducial_id), None)
                if vert is None:
                    rospy.logfatal("Target fiducial not found")
                    return
                x = min(vert.x0, min(vert.x1, min(vert.x2, vert.x3)))
                y = min(vert.y0, min(vert.y1, min(vert.y2, vert.y3)))
                rospy.loginfo(f"x: {x}, y: {y}")
                while offset < 20:
                    index = int((snap.width * (round(x) - offset) + y - offset) * 3)
                    BGR = np.array([int(bit) for bit in \
                            snap.data[index:index + 3]]).astype(np.uint8)
                    rospy.loginfo(f"BGR: {int(BGR[0])} {int(BGR[1])} {int(BGR[2])}")
                    rospy.loginfo("Colour: " + self._colour_detector.detect_color(BGR))
                    offset += 1
                offset = 0
                """
                input()
            break


            self._colour = self._colour_detector.detect_color(BGR)
            if self._colour == 'black' or self._colour == 'white':
                self._colour = None
            offset += 1
        rospy.loginfo("Target colour is " + self._colour)

    def wait_move(self):
        while self.joint_code(None) == 1 and not rospy.is_shutdown():
            rospy.sleep(1)
        return self.joint_code(None)

    def wait_grip(self, set_grip):
        while self.is_grip(None) != set_grip and not rospy.is_shutdown():
            rospy.sleep(1)

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
            if len(scan) > 1:
                i = 1 if scan[0].fiducial_id == BASE_FIDUCIAL else 0
                if self._target is None:
                    self._target = scan[i]
                    rospy.loginfo("Targetting fiducial id " \
                            f"{self._target.fiducial_id}")
                elif self._target.fiducial_id == scan[i].fiducial_id:
                    target_reach = np.linalg.norm(
                            [
                                self._target.transform.translation.x,
                                self._target.transform.translation.y,
                                self._target.transform.translation.z
                            ])
                    scan_reach = np.linalg.norm(
                            [
                                scan[i].transform.translation.x,
                                scan[i].transform.translation.y,
                                scan[i].transform.translation.z
                            ])
                    shift = abs((target_reach - scan_reach) / target_reach)
                    if shift < 0.01:
                        rospy.loginfo("Fiducial marker " \
                                f"{self._target.fiducial_id} ready")
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
        pos = self._tfBuffer.lookup_transform(
                f"fiducial_{BASE_FIDUCIAL}",
                f"fiducial_{self._target.fiducial_id}",
                rospy.Time()).transform.translation
        pos.z -= 0.305
        rot = None
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
            if "drop" not in zone:
                continue
            if spec["colour"] == self._colour:
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
