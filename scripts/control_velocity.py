#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
from pid import PID

class control_velocity(object):
    def __init__(self):
        self.wheel_track = rospy.get_param('wheel_track', 0.125)
        self.wheel_diameter = rospy.get_param('wheel_diameter', 0.04)

        self.left_vel = 0
        self.right_vel = 0

        self.linear_pid = PID(165, 140, 0, 125.83)
        self.angular_pid = PID(5, 4, 0, 3.125)

        self.linear_pid.setpoint = 0
        self.angular_pid.setpoint = 0

        self.linear_sign = 1
        self.summer = {-1: 0, 1: 0}

        self.left_pub = rospy.Publisher('/wheel_left/duty', Int16, queue_size=1)
        self.right_pub = rospy.Publisher('/wheel_right/duty', Int16, queue_size=1)

        rospy.Subscriber("/cmd_vel", Twist, self.update_setpoints)
        rospy.Subscriber("/wheel_odom", Odometry, self.update_duty)

    def update_setpoints(self, cmd_vel):
        self.linear_pid.setpoint = cmd_vel.linear.x
        self.angular_pid.setpoint = cmd_vel.angular.z

    def update_duty(self, odom):
        twist = odom.twist.twist
        sign = signum(twist.linear.x)

        if sign != 0 and self != self.linear_sign:
            self.angular_pid.clear_integrator()
            self.linear_sign = sign

        linear = self.linear_pid.calc(twist.linear.x)
        angular = self.angular_pid.calc(twist.angular.z)

        self.left_pub.publish(Int16(-linear + angular))
        self.right_pub.publish(Int16(-linear - angular))

def signum(n):
    if n > 0:
        return 1
    if n < 0:
        return -1
    return 0

if __name__ == '__main__':
    rospy.init_node('control_velocity')
    control_velocity()
    rospy.spin()
