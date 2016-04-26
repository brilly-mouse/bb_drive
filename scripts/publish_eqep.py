#!/usr/bin/env python
import rospy
from math import pi, cos, sin
from eqep import eQEP
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, PoseStamped
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
rospy.init_node('publish_eqep')
pub_left = rospy.Publisher('/wheel_left/state', Float32, queue_size=1)
pub_right = rospy.Publisher('/wheel_right/state', Float32, queue_size=1)
pub_twist = rospy.Publisher('/twist', Twist, queue_size=1)
pub_pose = rospy.Publisher('/pose', PoseStamped, queue_size=1)

# initialiaze eQEPs
eqep_left = eQEP(eQEP.eQEP2, eQEP.MODE_RELATIVE)
eqep_right = eQEP(eQEP.eQEP0, eQEP.MODE_RELATIVE)

# set update frequency
frequency = rospy.get_param('frequency', 30)
period = 1000000000 / frequency
eqep_left.set_period(period)
eqep_right.set_period(period)

# other params
ticks_per_rev = rospy.get_param('ticks_per_rev', 360)
wheel_track = rospy.get_param('wheel_track', 0.108)
wheel_diameter = rospy.get_param('wheel_diameter', 0.04)

theta = 0
x = 0
y = 0

count = 0

prev_time = rospy.get_time()

while not rospy.is_shutdown():
    # get raw tick count
    left = eqep_left.poll_position()
    right = eqep_right.get_position()

    curr_time = rospy.get_time()
    dT = curr_time - prev_time
    prev_time = curr_time

    # _super_ sketchy fix for some nondeterminism issue
    # with our magnetic encoders
    if left == -1:
        left = 0
    if right == -1:
        right = 0

    # ticks => rad/sec left *= pi * frequency / ticks_per_rev
    left *= pi / dT / ticks_per_rev
    right *= pi / dT / ticks_per_rev * -1

    # publish wheel speeds
    pub_left.publish(Float32(left))
    pub_right.publish(Float32(right))

    # rad/sec => m/sec
    left *= wheel_diameter / 2
    right *= wheel_diameter / 2

    # publish twist
    twist = Twist()
    twist.linear.x = (right + left) / 2
    twist.angular.z = (right - left) / wheel_track
    pub_twist.publish(twist)

    # update pose
    theta += twist.angular.z * dT
    x += twist.linear.x * cos(theta) * dT
    y += twist.linear.x * sin(theta) * dT

    # publish pose
    p = PoseStamped()
    p.pose.position.x = x
    p.pose.position.y = y
    # p.pose.orientation.x = 0
    # p.pose.orientation.y = 0
    # p.pose.orientation.z = 1
    # p.pose.orientation.w = cos(theta / 2)
    q = transformations.quaternion_from_euler(0, 0, theta)
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    p.header.stamp = rospy.get_rostime()
    p.header.seq = count
    p.header.frame_id = '1'
    pub_pose.publish(p)

    count += 1
