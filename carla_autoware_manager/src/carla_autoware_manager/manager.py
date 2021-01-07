#!/usr/bin/env python
"""
"""
import os

import rospy
import rospkg
import roslaunch

from carla_autoware_manager.srv import StartBridge, StopBridge, StartAgent, StopAgent

def dummy_handler(): pass
roslaunch.pmon._init_signal_handlers = dummy_handler

class CarlaAutowareManager(object):

    def __init__(self):
        self._bridge_launch = None
        self._agent_launch = None

        self.start_bridge_srv = rospy.Service("start_bridge", StartBridge, self.start_bridge)
        self.stop_bridge_srv = rospy.Service("stop_bridge", StopBridge, self.stop_bridge)
        self.start_agent_srv = rospy.Service("start_agent", StartAgent, self.start_agent)
        self.stop_agent_srv = rospy.Service("stop_agent", StopAgent, self.stop_agent)

        self._rospack = rospkg.RosPack()

    def _get_package_path(self, package_name):
        try:
            return self._rospack.get_path(package_name)
        except Exception:
            return None

    def start_bridge(self, req):
        rospy.loginfo("Starting ROS Bridge...")
        package_path = self._get_package_path("carla_ros_bridge")
        if package_path is None:
            return False

        launch_file = os.path.join(package_path, "launch", "carla_ros_bridge_with_example_ego_vehicle.launch") 
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self._bridge_launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self._bridge_launch.start()
        return True

    def stop_bridge(self, req):
        if self._bridge_launch is not None:
            self._bridge_launch.shutdown()
            return True
        return False

    def start_agent(self, req):
        rospy.loginfo("Starting agent...")
        package_path = self._get_package_path("carla_autoware_agent")
        if package_path is None:
            return False

        launch_file = os.path.join(package_path, "launch", "base.launch") 
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self._agent_launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self._agent_launch.start()
        return True

    def stop_agent(self, req):
        if self._agent_launch is not None:
            self._agent_launch.shutdown()
            return True
        return False


def main():
    """
    main function
    """
    rospy.init_node("carla_autoware_manager")
    manager = CarlaAutowareManager()
    rospy.spin()

if __name__ == '__main__':
    main()
