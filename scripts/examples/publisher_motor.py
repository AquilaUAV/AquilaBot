#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from time import time
from math import sin

max_voltage = 255
omega_1 = 0.9
omega_2 = 0.5
phi_1 = 2
phi_2 = 5


def publisher_motor():
    motor_cmd_left = rospy.Publisher('/omegabot/cmd/motor/left', Int16, queue_size=1)
    motor_cmd_right = rospy.Publisher('/omegabot/cmd/motor/right', Int16, queue_size=1)
    rospy.init_node('motor_cmd', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        control = (max_voltage*sin(omega_1*time() + phi_1), max_voltage*sin(omega_2*time() + phi_2))
        motor_cmd_left.publish(control[0])
        motor_cmd_right.publish(control[1])

        rospy.loginfo(control)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher_motor()
    except rospy.ROSInterruptException:
        pass
