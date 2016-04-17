#!/usr/bin/env python
import rospy
from math import pi
from eqep import eQEP
from std_msgs.msg import Float32

#
# this is probably fine for encoders for now but there are some
# inherent granularity issues with using the eQEPs for velocity
# control; probably want to switch to PRUs at some point
#


# ros stuff
rospy.init_node('publish_eqep')
pub_left = rospy.Publisher('/wheel_left/state', Float32, queue_size=1)
pub_right = rospy.Publisher('/wheel_right/state', Float32, queue_size=1)

# initialiaze eQEPs
eqep_left = eQEP(eQEP.eQEP0, eQEP.MODE_RELATIVE)
eqep_right = eQEP(eQEP.eQEP2, eQEP.MODE_RELATIVE)

# set update frequency
frequency = rospy.get_param('frequency', 30)
period = 1000000000 / frequency
eqep_left.set_period(period)
eqep_right.set_period(period)

# other params
ticks_per_rev = rospy.get_param('ticks_per_rev', 360)

rospy.loginfo("Starting publish for encoder data...")

while not rospy.is_shutdown():
    # get raw tick count
    left = eqep_left.poll_position()
    right = eqep_right.get_position()

    # publish in rad/sec
    pub_left.publish(Float32(left * pi * frequency / ticks_per_rev) * -1)
    pub_right.publish(Float32(right * pi * frequency / ticks_per_rev))

rospy.loginfo("Exiting...")
