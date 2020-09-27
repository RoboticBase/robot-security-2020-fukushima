#!/usr/bin/env python
import random

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import NavSatFix, NavSatStatus


def send():
    rospy.init_node('send_position', anonymous=True)
    pub = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
    r = rospy.Rate(3)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'base_link'

        status = NavSatStatus()
        status.status = -1
        status.service = 1

        nav = NavSatFix()
        nav.header = h
        nav.status = status
        nav.latitude = 35.0 + random.random()
        nav.longitude = 140.0 + random.random()
        nav.altitude = 101.0 + random.random()
        nav.position_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        nav.position_covariance_type = 0

        pub.publish(nav)
        rospy.loginfo('published %s', nav)
        r.sleep()


if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
