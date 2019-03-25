#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
import sys
import glob
import os

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('**/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================

class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)

# ==============================================================================
# -- LidarSensor ---------------------------------------------------------------
# ==============================================================================

class LidarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('role_name', 'sensor')
        bp.set_attribute('range', '5000')
        bp.set_attribute('channels', '32')
        bp.set_attribute('points_per_second', '320000')
        bp.set_attribute('upper_fov', '2.0')
        bp.set_attribute('lower_fov', '-26.8')
        bp.set_attribute('rotation_frequency', '20')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.0, z=2.4)), attach_to=self._parent)

# ==============================================================================
# -- FrontCameraSensor ---------------------------------------------------------
# ==============================================================================

class FrontCamera(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('role_name', 'front')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=2.0, z=2.0)), attach_to=self._parent)

