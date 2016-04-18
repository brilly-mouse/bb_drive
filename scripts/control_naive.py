#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from pid import PID

class control_naive(object):
    def __init__(self):
        self.wheel_track = rospy.get_param('wheel_track', 0.125)
        self.wheel_diameter = rospy.get_param('wheel_diameter', 0.04)

        self.left_pid = PID(4, 0, 0, 3)
        self.right_pid = PID(4, 0, 0, 3)

        self.left_set_pub = rospy.Publisher('/wheel_left/setpoint', Float32, queue_size=1)
        self.right_set_pub = rospy.Publisher('/wheel_right/setpoint', Float32, queue_size=1)

        self.left_pub = rospy.Publisher('/wheel_left/duty', Int16, queue_size=1)
        self.right_pub = rospy.Publisher('/wheel_right/duty', Int16, queue_size=1)

        rospy.Subscriber("/cmd_vel", Twist, self.update_setpoints)
        rospy.Subscriber("/wheel_left/state", Float32, self.create_cb(self.left_pid, self.left_pub))
        rospy.Subscriber("/wheel_right/state", Float32, self.create_cb(self.right_pid, self.right_pub))

    def update_setpoints(self, cmd_vel):
        # twist => m/sec for each wheel
        left  = cmd_vel.linear.x - cmd_vel.angular.z * self.wheel_track / 2
        right = cmd_vel.linear.x + cmd_vel.angular.z * self.wheel_track / 2

        # m/sec => rad/sec & assign as control loop setpoints
        self.left_pid.setpoint = left / self.wheel_diameter * 2
        self.right_pid.setpoint = right / self.wheel_diameter * 2

        # publish setpoint
        self.left_set_pub.publish(Float32(self.left_pid.setpoint))
        self.right_set_pub.publish(Float32(self.right_pid.setpoint))

    def create_cb(self, pid, publisher):
        def cb(data):
            duty = min(100, max(-100, pid.calc(data.data)))
            publisher.publish(Int16(duty))
        return cb

if __name__ == '__main__':
    rospy.init_node('control_naive')
    control_naive()
    rospy.spin()
