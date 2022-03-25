"""
2022.03.25 Kong Ji Kang
brrandon13@gist.ac.kr


client = carla.Client(args.host, args.port)
client.set_timeout(20.0)

sim_world = client.get_world() # SERVER

1. get target_id(int) from p2p communitation

2. initialize World(carla_world, args)
self.world = carla_world
self.map = self.world.get_map() -> to get spawnpoints(waypoints)
self.actor = None
self.sensor_manager = None

self.actor = self.world.get_actor(target_id)
-> initialize sensor attached to target_vehicle

"""



import glob
import os
import sys

from pygame import K_q
from pygame import KMOD_CTRL

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

import can
import cantools

import pygame
from pygame.locals import K_DOWN
from pygame.locals import K_ESCAPE

import numpy as np

from agents.navigation.can_package import CAN

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world):
        self.world = carla_world
        self.map = self.world.get_map()
        self.actor = None
        self.sensor_manager = None

        self.start()

    def start(self):
        target_id = 0
        # get target_id(int) 
        self.actor = self.world.get_actor(target_id)  # carla.Vehicle
        pass


class SensorManager(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

def game_loop(args):
    pygame.init()
    pygame.font.init()
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(20.0)

        sim_world = client.get_world()  # CLIENT
        # get_world(self):
        # Returns the world object currently active in the simulation.
        # This world will be later used for example to load maps.

        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()

            # set the world to synchronous mode
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.02 
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        while True:
            pass
            

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='960x540',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='vehicle.*',
        help='actor filter (default: "vehicle.*")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':
    main()
