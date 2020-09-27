#!/usr/bin/env python
import json
import signal
from datetime import datetime, timedelta, timezone
from threading import Lock

import rospy
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix, BatteryState
from eams_msgs.msg import State

from proton.reactor import Container

from producer import Producer
from utils import wrap_namespace
import message_filters_py3 as message_filters


class RobotState:
    def __init__(self, producer):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._producer = producer
        self._prev_ms = datetime.now(timezone.utc)
        self._lock = Lock()

    def state_cb(self, position, compass, battery):
        rospy.loginfo('subscribe a state message, position=%s, compass=%s, battery=%s', position, compass, battery)
        now = datetime.now(timezone.utc)
        if now >= self._prev_ms + timedelta(milliseconds=self._params.thresholds.send_delta_ms) and self._lock.acquire(False):
            self._prev_ms = now
            message = {
                'time': datetime.fromtimestamp(position.header.stamp.to_time(), timezone.utc).isoformat(),
                'pose': {
                    'point': {
                        'latitude': position.latitude,
                        'longitude': position.longitude,
                        'altitude': position.altitude,
                    },
                    'angle': {
                        'yaw': compass.data,
                    },
                },
                'covariance': list(position.position_covariance),
                'battery': {
                    'voltage': battery.voltage,
                    'current': battery.current,
                }
            }
            self._producer.send(json.dumps({
                'attrs': message,
            }))
            self._lock.release()


class MissionState:
    def __init__(self, producer):
        self._producer = producer

    def state_cb(self, state):
        rospy.loginfo('subscribe a mission state message, %s', state)
        message = {
            'time': datetime.fromtimestamp(state.header.stamp.to_time(), timezone.utc).isoformat(),
            'mode': 'init' if state.status == 0 else 'navi' if state.status == 1 else 'standby' if state.status == 2 else 'error',
        }
        self._producer.send(json.dumps({
            'attrs': message,
        }))


class NaviResult:
    def __init__(self, producer):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._producer = producer

    def navi_result_cb(self, result):
        rospy.loginfo('subscribe a navi result, %s', result)
        self._producer.send(result.data)


def main():
    rospy.init_node('eams_producer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    producer = Producer()

    mission_state = MissionState(producer)
    rospy.Subscriber(params.topic.mission_state, State, mission_state.state_cb)

    robot_state = RobotState(producer)
    position_sub = message_filters.Subscriber(params.topic.position, NavSatFix)
    compass_sub = message_filters.Subscriber(params.topic.compass, Float64)
    battery_sub = message_filters.Subscriber(params.topic.battery, BatteryState)

    slop = float(params.thresholds.slop_ms)/1000.0
    ts = message_filters.ApproximateTimeSynchronizer([position_sub, compass_sub, battery_sub], 10, slop, allow_headerless=True)
    ts.registerCallback(robot_state.state_cb)

    navi_result = NaviResult(producer)
    rospy.Subscriber(params.topic.navi_cmdexe, String, navi_result.navi_result_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        producer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(producer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
