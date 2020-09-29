#!/usr/bin/env python
from datetime import datetime

import rospy
from std_msgs.msg import Header
from eams_msgs.msg import ImageInfo


def send():
    rospy.init_node('send_imageinfo', anonymous=True)
    pub = rospy.Publisher('/camera/image_info', ImageInfo, queue_size=10)
    r = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = 'image_info'

        image_info = ImageInfo()
        image_info.header = h
        image_info.time = datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')
        image_info.lat = '35.878648044793174'
        image_info.lng = '140.96813260913413'
        image_info.yaw = '10.136800372575802'
        image_info.hash = '5fffcc3dfb4858a913c8353d7943eba94129cd327c711a3ebb47663e097d1bfa'
        image_info.path = '/foo/bar/buz.jpg'

        pub.publish(image_info)
        rospy.loginfo('published image_info %s', image_info)
        r.sleep()


if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
