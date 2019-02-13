#!/usr/bin/env python

# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Carla client for usage with Autoware
# derived from manual_control.py

"""
Welcome to CARLA autoware control.
"""

from __future__ import print_function

from time import sleep
import signal
import sys
import argparse
import glob
import logging
import os
import random

try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

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
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, actor_filter, actor_spawnpoint):
        self.world = carla_world
        self.player = None
        self.gnss_sensor = None
        self.lidar_sensor = None
        self.front_camera = None
        self.view_camera = None
        self._actor_filter = actor_filter
        self._actor_spawnpoint = actor_spawnpoint
        self.restart()

    def restart(self):
        # Get a random blueprint.
        blueprint = random.choice(self.world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', 'hero')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        # Spawn the player.
        if self._actor_spawnpoint:
            spawn_point = carla.Transform()
            spawn_point.location.x = self._actor_spawnpoint[0]
            spawn_point.location.y = self._actor_spawnpoint[1]
            spawn_point.location.z = self._actor_spawnpoint[2]
            spawn_point.rotation.yaw = self._actor_spawnpoint[3]
            if self.player is not None:
                self.destroy()
            while self.player is None:
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        else:
            if self.player is not None:
                spawn_point = self.player.get_transform()
                spawn_point.location.z += 2.0
                spawn_point.rotation.roll = 0.0
                spawn_point.rotation.pitch = 0.0
                self.destroy()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            while self.player is None:
                spawn_points = self.world.get_map().get_spawn_points()
                spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
                self.player = self.world.try_spawn_actor(blueprint, spawn_point)
        # Set up the sensors.
        self.gnss_sensor = GnssSensor(self.player)
        self.lidar_sensor = LidarSensor(self.player)
        self.front_camera = FrontCamera(self.player)
        self.view_camera = ViewCamera(self.player)

    def destroy(self):
        actors = [
            self.view_camera.sensor,
            self.front_camera.sensor,
            self.gnss_sensor.sensor,
            self.lidar_sensor.sensor,
            self.player]

        for actor in actors:
            if actor is not None:
                actor.destroy()


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
        bp.set_attribute('range', '5000')
        bp.set_attribute('role_name', 'sensor')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.0, z=2.0)), attach_to=self._parent)

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

# ==============================================================================
# -- CameraSensor for Visualization --------------------------------------------
# ==============================================================================

class ViewCamera(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('role_name', 'viewFromBehind')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15)), attach_to=self._parent)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================

def sigterm_handler(_signo, _stack_frame):
    # Raises SystemExit(0):
    sys.exit(0)

def game_loop(args):
    world = None
    signal.signal(signal.SIGTERM, sigterm_handler)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        world = World(client.get_world(), args.filter, args.spawnpoint)

        while True:
            sleep(1)    

    finally:

        if world is not None:
            world.destroy()



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Autoware Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--spawnpoint',
        nargs=4,
        metavar='FLOAT',
        type=float,
        help='spawn point (X Y Z YAW, default: random predefined spawnpoints)')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
