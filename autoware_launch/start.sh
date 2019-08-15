#!/bin/bash -e

# it is assumed, that CARLA is already running


#there is a bug in Autoware, that prevents the stack from starting with very low timestamps
#https://github.com/autowarefoundation/autoware/issues/2200
#
#cleanup from previous runs
rosnode cleanup <<< 'y'

#As a workaround we wait 5 seconds in simulation time
echo "Waiting for 5 seconds in simulation time..."
python << EOF
import sys
import rospy
from rosgraph_msgs.msg import Clock

def on_clock(data):
    if data.clock.to_sec() >= 5:
        rospy.signal_shutdown("")

rospy.init_node("sleep", anonymous=True)
subscriber = rospy.Subscriber("/clock", Clock, on_clock)
rospy.spin()
EOF
echo "Starting Autoware."

roslaunch $CARLA_AUTOWARE_ROOT/challenge.launch

exit $?
