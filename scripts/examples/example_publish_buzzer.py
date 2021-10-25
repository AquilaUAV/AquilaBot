#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from time import time
from math import sin

min_pwm = 0
max_pwm = 255
omega = 0.5
phi = 2


def publisher_motor():
    buzzer_cmd = rospy.Publisher('/omegabot/cmd/buzzer', Int16, queue_size=1)
    rospy.init_node('buzzer_cmd', anonymous=True)
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():

        avg_pwm = (max_pwm + min_pwm) / 2
        control = int(round(avg_pwm*sin(omega*time() + phi) + min_pwm + avg_pwm))
        buzzer_cmd.publish(control)

        rospy.loginfo(control)
        rate.sleep()


if __name__ == '__main__':
    try:
        publisher_motor()
    except rospy.ROSInterruptException:
        pass
