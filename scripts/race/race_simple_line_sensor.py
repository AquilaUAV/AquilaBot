#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

motor_cmd_left = rospy.Publisher('/omegabot/cmd/motor/left', Int16, queue_size=1)
motor_cmd_right = rospy.Publisher('/omegabot/cmd/motor/right', Int16, queue_size=1)

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


white = [830, 780]
black = [980, 930]
max_speed = 100
angle_ku = 1.0
def line_sensor_callback(line_sensor_data):
    for i in range(2):
        line_sensor_data[i] -= white[i]
        line_sensor_data[i] /= 1.0 * black[i] - white[i]
        if line_sensor_data[i] > 1.0:
            line_sensor_data[i] = 1.0
        if line_sensor_data[i] < 0.0:
            line_sensor_data[i] = 0.0
    angle_cmd = line_sensor_data[0] - line_sensor_data[1]
    angle_pwm = [-angle_cmd*max_speed*angle_ku, angle_cmd*max_speed*angle_ku]
    for i in range(2):
        angle_pwm[i] = int(round(angle_pwm[i]))
    angle_pwm_max = max(angle_pwm[0], angle_pwm[1])
    control = [max_speed - angle_pwm_max + angle_pwm[0], max_speed - angle_pwm_max + angle_pwm[1]]
    rospy.loginfo((angle_pwm, control))
    motor_cmd_left.publish(control[0])
    motor_cmd_right.publish(control[1])


def race_simple():
    global line_sensor, line_sensor_updated
    rospy.init_node('race_simple', anonymous=True)

    rospy.Subscriber("/omegabot/sensor/line/left", Int16, line_sensor_left_callback)
    rospy.Subscriber("/omegabot/sensor/line/right", Int16, line_sensor_right_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        race_simple()
    except rospy.ROSInterruptException:
        pass
