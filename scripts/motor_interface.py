#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
import Adafruit_BBIO.PWM as PWM

rospy.init_node('motor_interface')

left_f = rospy.get_param("left_motor_pin_f", "P8_13")
left_r = rospy.get_param("left_motor_pin_r", "P8_19")
right_f = rospy.get_param("right_motor_pin_f", "P9_16")
right_r = rospy.get_param("right_motor_pin_r", "P9_14")

for p in [left_f, left_r, right_f, right_r]:
    PWM.start(p, 0, 20000)

def create_cb(f, r):
    def cb(data):
        duty = min(100, max(-100, data.data))
        if duty > 0:
            PWM.set_duty_cycle(f, duty)
            PWM.set_duty_cycle(r, 0)
        else:
            PWM.set_duty_cycle(f, 0)
            PWM.set_duty_cycle(r, -duty)
    return cb

rospy.Subscriber("/wheel_left/duty", Int16, create_cb(left_f, left_r))
rospy.Subscriber("/wheel_right/duty", Int16, create_cb(right_f, right_r))

rospy.spin()

PWM.cleanup()
