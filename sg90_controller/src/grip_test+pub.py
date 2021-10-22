#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def grip_mover():
    pub = rospy.Publisher('/move_sg90', Bool, queue_size=10)
    rospy.init_node('grip_mover', anonymous=True)
    rate = rospy.Rate(0.5)
    
    #open/close gripper once per second
    close = True
    while not rospy.is_shutdown():
        if close:
            rospy.loginfo(close)
            close = False
        else:
            rospy.loginfo(close)
            close = True
        pub.publish(close)
        rate.sleep()


if __name__ == '__main__':
    try:
        grip_mover()
    except rospy.ROSInterruptException:
        pass

