#!/usr/bin/env python
import rospy
from math import pi, cos, sin
from eqep import eQEP
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from time import sleep
from overlay import Overlay
from tf import transformations

#
# this is probably fine for encoders for now but there are some
# inherent granularity issues with using the eQEPs for velocity
# control; probably want to switch to PRUs at some point
#
# overlay to device tree
Overlay.apply("bone_eqep0")
Overlay.apply("bone_eqep2b")

# ros stuff
rospy.init_node('odom_publisher')
pub_left = rospy.Publisher('/wheel_left/state', Float32, queue_size=1)
pub_right = rospy.Publisher('/wheel_right/state', Float32, queue_size=1)
pub_odom = rospy.Publisher('/wheel_odom', Odometry, queue_size=3)

# initialiaze eQEPs
eqep_left = eQEP(eQEP.eQEP2, eQEP.MODE_ABSOLUTE)
eqep_right = eQEP(eQEP.eQEP0, eQEP.MODE_ABSOLUTE)

# set update frequency
frequency = rospy.get_param('frequency', 30)
period = 1000000000 / frequency
eqep_left.set_period(period)
eqep_right.set_period(period)

# other params
ticks_per_rev = rospy.get_param('ticks_per_rev', 360)
wheel_track = rospy.get_param('wheel_track', 0.106)
wheel_diameter = rospy.get_param('wheel_diameter', 0.04)

theta = 0
x = 0
y = 0

count = 0

prev_time = rospy.get_time()
prev_left = eqep_left.get_position()
prev_right = eqep_left.get_position()
while not rospy.is_shutdown():
    # get raw tick count
    curr_left = eqep_left.poll_position()
    curr_right = eqep_right.get_position()
    curr_time = rospy.get_time()
    dT = curr_time - prev_time

    # ticks => rad/sec left *= pi * frequency / ticks_per_rev
    left = (curr_left - prev_left) * 2 * pi / dT / ticks_per_rev
    right = (curr_right - prev_right) * 2 * pi / dT / ticks_per_rev * -1

    # publish wheel speeds in rad/sec
    pub_left.publish(Float32(left))
    pub_right.publish(Float32(right))

    # rad/sec => m/sec
    left *= wheel_diameter / 2
    right *= wheel_diameter / 2

    # instantiate odom message & assign twist values
    odom = Odometry()
    odom.twist.twist.linear.x = (right + left) / 2
    odom.twist.twist.angular.z = (right - left) / wheel_track

    # update pose
    theta += odom.twist.twist.angular.z * dT
    x += odom.twist.twist.linear.x * cos(theta) * dT
    y += odom.twist.twist.linear.x * sin(theta) * dT

    # pose components
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    q = transformations.quaternion_from_euler(0, 0, theta)
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    # attach headers & publish
    odom.header.stamp = rospy.get_rostime()
    odom.header.seq = count
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'base_link'
    pub_odom.publish(odom)

    prev_left = curr_left
    prev_right = curr_right
    prev_time = curr_time
    count += 1
