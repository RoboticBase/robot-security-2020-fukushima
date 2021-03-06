#!/usr/bin/env python
import json
import signal
from datetime import datetime, timezone

import rospy

from std_msgs.msg import Header, String
from eams_msgs.msg import Control, Mission, Detail

from proton.reactor import Container

from consumer import Consumer
from utils import wrap_namespace


class Dispatcher:
    def __init__(self, naviCommand, alertCommand):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._naviCommand = naviCommand
        self._alertCommand = alertCommand

    def dispatch_cb(self, msg):
        rospy.loginfo('consume a command message, %s', msg)
        try:
            message = json.loads(msg)
            if 'cmd' not in message:
                rospy.logerr('invalid payload')
                return
            if self._params.rb.navi_cmd_name in message['cmd']:
                body = message['cmd'][self._params.rb.navi_cmd_name]
                ros_published_control, ros_published_mission = self._naviCommand.process(body)
                rospy.loginfo('processed the navi command, control=%s, mission=%s', ros_published_control, ros_published_mission)
            elif self._params.rb.alert_cmd_name in message['cmd']:
                body = message['cmd'][self._params.rb.alert_cmd_name]
                self._alertCommand.process(body)
            else:
                rospy.logerr('unknown command')
        except (ValueError, TypeError) as e:
            rospy.logerr('invalid payload, %s', e)


class AlertCommand:
    def __init__(self, alert_pub, cmdexe_pub):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._alert_pub = alert_pub
        self._cmdexe_pub = cmdexe_pub

    def process(self, body):
        alert_name_message = String()
        alert_name_message.data = body
        self._alert_pub.publish(alert_name_message)
        self._cmdexe_pub.publish(self._make_result(body))

    def _make_result(self, body):
        message = {}
        message[self._params.rb.alert_cmd_name] = {
            'time': datetime.fromtimestamp(rospy.Time.now().to_time(), timezone.utc).isoformat(),
            'received_alert': body,
        }
        result = String()
        result.data = json.dumps(message)
        return result


class NaviCommand:
    def __init__(self, control_pub, mission_pub, cmdexe_pub):
        self._params = wrap_namespace(rospy.get_param('~'))
        self._control_pub = control_pub
        self._mission_pub = mission_pub
        self._cmdexe_pub = cmdexe_pub
        self._mission_wait_ms = float(self._params.thresholds.mission_wait_ms)/1000.0
        self._waypoint_wait_ms = float(self._params.thresholds.waypoint_wait_ms)/1000.0

    def process(self, body):
        rospy.loginfo('process a navi message %s', body)

        control, mission, result = self._make_control_and_mission(body)
        if mission is not None:
            self._mission_pub.publish(mission)
            rospy.sleep(self._mission_wait_ms)
        if control is not None:
            self._control_pub.publish(control)
        if result is not None:
            self._cmdexe_pub.publish(result)
        return control, mission

    def _make_control_and_mission(self, body):
        if 'command' not in body or body['command'] not in ('start', 'stop', 'suspend', 'resume'):
            return None, None, None

        result = self._make_result(body)
        cmd = body['command']

        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = cmd
        control = Control()
        control.header = h
        if cmd == 'start':
            control.command = 1
            mission = self._make_mission(body)
            return control, mission, result
        elif cmd == 'stop':
            control.command = 0
            return control, None, result
        elif cmd == 'suspend':
            control.command = 2
            return control, None, result
        elif cmd == 'resume':
            control.command = 3
            return control, None, result
        else:
            rospy.logerr('invalid command {}'.format(body['command']))
            return None, None, None

    def _make_mission(self, body):
        mission = Mission()
        mission.header.stamp = rospy.Time.now()
        mission.header.frame_id = 'mission'

        details = []
        for wp in body.get('waypoints', []):
            # Create a waypoint message.
            waypoint = Detail()
            waypoint.command = 1
            waypoint.lat = wp['point']['latitude']
            waypoint.lng = wp['point']['longitude']
            waypoint.param1 = wp['speed'] if wp.get('speed') else 0.0
            waypoint.param2 = 0.0
            waypoint.param3 = 0.0
            waypoint.param4 = 0.0
            details.append(waypoint)
            # Create an angle message.
            if 'theta' in wp['angle'] and wp['angle']['theta'] is not None:
                theta = Detail()
                theta.command = 3
                theta.lat = 0.0
                theta.lng = 0.0
                theta.param1 = wp['angle']['theta']
                theta.param2 = 0.0
                theta.param3 = 0.0
                theta.param4 = 0.0
                details.append(theta)
            # Create a map message.
            if 'metadata' in wp and 'map' in wp['metadata'] and wp['metadata']['map'] in ('GPS', 'Cartographer'):
                mp = Detail()
                mp.lat = 0.0
                mp.lng = 0.0
                mp.param1 = 0.0
                mp.param2 = 0.0
                mp.param3 = 0.0
                mp.param4 = 0.0
                if wp['metadata']['map'] == 'GPS':
                    mp.command = 4
                    details.append(mp)
                elif wp['metadata']['map'] == 'Cartographer':
                    mp.command = 5
                    details.append(mp)
                else:
                    rospy.logerr('invalid map command {}'.format(wp['metadata']['map']))
            # Create a delay message.
            delay = Detail()
            delay.command = 2
            delay.lat = 0.0
            delay.lng = 0.0
            delay.param1 = wp['metadata']['delay'] if 'metadata' in wp and wp['metadata'].get('delay') else 0.0
            delay.param2 = 0.0
            delay.param3 = 0.0
            delay.param4 = 0.0
            details.append(delay)
        mission.details = details[:-1]
        return mission

    def _make_result(self, body):
        message = {}
        message[self._params.rb.navi_cmd_name] = {
            'time': datetime.fromtimestamp(rospy.Time.now().to_time(), timezone.utc).isoformat(),
            'received_time': body['time'],
            'received_command': body['command'],
            'received_waypoints': body['waypoints'],
            'result': 'ack',
            'errors': [],
        }
        result = String()
        result.data = json.dumps(message)
        return result


def main():
    rospy.init_node('amqpconsumer', anonymous=True, disable_signals=True)
    params = wrap_namespace(rospy.get_param('~'))

    control_pub = rospy.Publisher(params.topic.control_cmd, Control, queue_size=1)
    mission_pub = rospy.Publisher(params.topic.mission_cmd, Mission, queue_size=1)
    navi_cmdexe_pub = rospy.Publisher(params.topic.navi_cmdexe, String, queue_size=1)
    alert_cmdexe_pub = rospy.Publisher(params.topic.alert_cmdexe, String, queue_size=1)
    alert_pub = rospy.Publisher(params.topic.alert, String, queue_size=1)

    naviCommand = NaviCommand(control_pub, mission_pub, navi_cmdexe_pub)
    alertCommand = AlertCommand(alert_pub, alert_cmdexe_pub)
    dispatcher = Dispatcher(naviCommand, alertCommand)
    consumer = Consumer(dispatcher.dispatch_cb)

    def handler(signum, frame):
        rospy.loginfo('shutting down...')
        consumer.shutdown()
    signal.signal(signal.SIGINT, handler)

    Container(consumer).run()
    rospy.signal_shutdown('finish')


if __name__ == '__main__':
    main()
