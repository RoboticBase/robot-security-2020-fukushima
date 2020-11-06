#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient


def play(sound_name):
    soundhandle = SoundClient()

    params = rospy.get_param('~')
    volume = float(params['volume'])
    sleep_time = int(params['sleep_time'])
    rospy.sleep(sleep_time)
    rospy.loginfo('Playing {}'.format(sound_name))
    alert_base_path = params['alert_base_path']
    soundhandle.playWave("{}{}.ogg".format(alert_base_path, sound_name), volume)
    rospy.sleep(sleep_time)


if __name__ == '__main__':
    alert_play_topic = rospy.get_param('/alert_player/topic/alert_play')
    rospy.init_node('alert_player', anonymous=True)
    rospy.Subscriber(alert_play_topic, String, play)
    rospy.spin()
