#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import time

global line_sensor, line_sensor_updated
line_sensor = [0, 0]
line_sensor_updated = [False, False]


def line_sensor_left_callback(data):
    global line_sensor, line_sensor_updated
    line_sensor_updated[0] = True
    line_sensor[0] = data.data
    if line_sensor_updated[0] and line_sensor_updated[1]:
        line_sensor_updated = [False, False]
        line_sensor_callback(line_sensor)


def line_sensor_right_callback(data):
    global line_sensor, line_sensor_updated
    line_sensor_updated[1] = True
    line_sensor[1] = data.data
    if line_sensor_updated[0] and line_sensor_updated[1]:
        line_sensor_updated = [False, False]
        line_sensor_callback(line_sensor)


def line_sensor_callback(line_sensor_data):
    rospy.loginfo(line_sensor_data)


def line_sensor_listener():
    global line_sensor, line_sensor_updated
    rospy.init_node('line_sensor_listener', anonymous=True)

    rospy.Subscriber("/omegabot/sensor/line/left", Int16, line_sensor_left_callback)
    rospy.Subscriber("/omegabot/sensor/line/right", Int16, line_sensor_right_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    line_sensor_listener()
