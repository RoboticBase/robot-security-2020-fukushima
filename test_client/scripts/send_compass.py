#!/usr/bin/env python
import random

import rospy
from std_msgs.msg import Float64


def send():
    rospy.init_node('send_compass', anonymous=True)
    pub = rospy.Publisher('/mavros/global_position/compass_hdg', Float64, queue_size=10)
    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        d = 10.0 + random.random()
        pub.publish(d)
        rospy.loginfo('published %f', d)
        r.sleep()


if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
