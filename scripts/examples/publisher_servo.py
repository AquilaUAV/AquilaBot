#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from time import time
from math import sin

min_angle = 20
max_angle = 120
omega_1 = 0.9
omega_2 = 0.5
phi_1 = 2
phi_2 = 5


def publisher_motor():
    servo_cmd = [rospy.Publisher('/omegabot/cmd/servo/{0}'.format(i), Int16, queue_size=1) for i in range(1, 2+1)]
    rospy.init_node('servo_cmd', anonymous=True)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        avg_angle = (max_angle + min_angle) / 2
        control = (avg_angle*sin(omega_1*time() + phi_1) + min_angle + avg_angle, avg_angle*sin(omega_2*time() + phi_2) + min_angle + avg_angle)

        for i in range(2):
            servo_cmd[i].publish(control[i])

        rospy.loginfo(control)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher_motor()
    except rospy.ROSInterruptException:
        pass
