#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from eams_msgs.msg import State


def send():
    rospy.init_node('send_mission_state', anonymous=True)
    pub = rospy.Publisher('/mission/state', State, queue_size=10, latch=True)
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'mission_state'

        s = State()
        s.header = h
        s.current = 1
        s.total = 5
        s.status = 1

        pub.publish(s)
        rospy.loginfo('published state %s', s)
        r.sleep()


if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
