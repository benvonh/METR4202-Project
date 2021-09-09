#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from ximea import xiapi
import cv2

import numpy as np


"""Returns numpy array of image data."""
def capture_frame(cam, img):
    cam.get_image(img)
    return img.get_image_data_numpy()

def main():
    # Initialise ximea components
    rospy.loginfo("Opening camera...")
    cam = xiapi.Camera()
    cam.open_device()
    cam.set_exposure(20000)
    cam.start_acquisition()
    img = xiapi.Image()

    # Initialise ROS components
    pub = rospy.Publisher("/test", String, queue_size=10)
    rospy.init_node("main")
    rate = rospy.Rate(5)
    
    # Main loop
    while not rospy.is_shutdown():
        data = capture_frame(cam, img)
        s = "test"
        rospy.loginfo(s)
        rospy.loginfo(np.array2string(data))
        pub.publish(s)
        rate.sleep()
    
    # Close camera
    rospy.loginfo("Closing camera...")
    cam.stop_acquisition()
    cam.close_device()



if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

