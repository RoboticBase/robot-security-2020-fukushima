#!/usr/bin/env python
import random

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState


def send():
    rospy.init_node('send_battery', anonymous=True)
    pub = rospy.Publisher('/mavros/battery', BatteryState, queue_size=10)
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        h = Header()
        h.stamp = rospy.Time.now()
        h.frame_id = ''

        battery = BatteryState()
        battery.header = h
        battery.voltage = 25 + random.random()
        battery.current = 0.0
        battery.charge = 0.0
        battery.capacity = 0.0
        battery.design_capacity = 0.0
        battery.percentage = 1.0
        battery.power_supply_status = 2
        battery.power_supply_health = 0
        battery.power_supply_technology = 0
        battery.present = True
        battery.cell_voltage = [battery.voltage]
        battery.location = 'id0'
        battery.serial_number = ''

        pub.publish(battery)
        rospy.loginfo('published battery %s', battery)
        r.sleep()


if __name__ == '__main__':
    try:
        send()
    except rospy.ROSInterruptException:
        pass
