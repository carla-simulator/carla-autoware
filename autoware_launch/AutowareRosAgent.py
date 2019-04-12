#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#

import os
import json
import numpy
from srunner.challenge.autoagents.ros_agent import RosAgent

class AutowareRosAgent(RosAgent):
    """
    Agent for Autoware ROS stack
    """

    def sensors(self):
        """
        The sensors required for Autoware
        """
        path = None
        if "TEAM_CODE_ROOT" in os.environ:
            path = os.environ["TEAM_CODE_ROOT"]
        if path is None:
            raise RuntimeError("Could not read sensor-definition") 
        filename = "{}/sensors.json".format(path)
        if not os.path.exists(filename):
            raise RuntimeError("Could not read sensor-definition from {}".format(filename))

        json_sensors = None
        with open(filename) as handle:
            json_sensors = json.loads(handle.read())
        
        return json_sensors["sensors"]

