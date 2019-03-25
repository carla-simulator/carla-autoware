#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
## receive carla_ros_bridge::CarlaEgoVehicleStatus and publishes autoware_msgs::VehicleStatus

import math
import rospy
from std_msgs.msg import Header
from carla_ros_bridge.msg import CarlaEgoVehicleInfo
from carla_ros_bridge.msg import CarlaEgoVehicleStatus
from autoware_msgs.msg import VehicleStatus

pub = rospy.Publisher('vehicle_status', VehicleStatus, queue_size=1)
vehicle_info = None

def vehicle_info_callback(data):
    global vehicle_info
    vehicle_info = data

def vehicle_status_callback(data):
    global vehicle_info
    if vehicle_info is None:
        return
    status = VehicleStatus();
    status.header = data.header
    status.speed = data.velocity

    #calculate max steering angle
    max_steering_angle = math.radians(70)
    # get max steering angle (use smallest non-zero value of all wheels)
    for wheel in vehicle_info.wheels:
        if wheel.steer_angle:
            if wheel.steer_angle and wheel.steer_angle < max_steering_angle:
                max_steering_angle = wheel.steer_angle

    status.angle = data.control.steer * math.degrees(max_steering_angle)
    status.speed = data.velocity * 3.6 # speed is expected in km/h
    status.gearshift = data.control.gear
    if data.control.manual_gear_shift:
        status.drivemode = VehicleStatus.MODE_MANUAL
    else:
        status.drivemode = VehicleStatus.MODE_AUTO
    pub.publish(status);

def convert_status_carla_to_autoware():
    rospy.init_node('convert_status_carla_to_autoware', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, vehicle_status_callback)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_info", CarlaEgoVehicleInfo, vehicle_info_callback)
    rospy.spin()

if __name__ == '__main__':
    convert_status_carla_to_autoware()
