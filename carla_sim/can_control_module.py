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

import socket
from threading import Thread

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
        self.actor_id = None

        self.get_target_id()

        self.start()

    def get_target_id(self):
        s = socket.socket()
        print('connecting to server...')
        s.connect(('localhost',8000))
        self.actor_id = int(s.recv(24).decode(encoding="utf-8"))

    def start(self):
        # get target_id(int) 
        self.actor = self.world.get_actor(self.actor_id)  # carla.Vehicle
        print(f'actor acquired: {self.actor.type_id}')

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
                settings.fixed_delta_seconds = 0.03
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)
            
        world = World(sim_world)  # WORLD
        agent = CAN(world.actor) # get actor from can_control_by_agent.py

        # Set the agent destination
        spawn_points = world.map.get_spawn_points()
        destination = random.choice(spawn_points).location
        agent.set_destination(destination)

        clock = pygame.time.Clock()

        while True:
            clock.tick()
            if args.sync:
                world.world.tick()
            else:
                world.world.wait_for_tick()

            if agent.done():
                agent.set_destination(random.choice(spawn_points).location)
                # world.hud.notification("The target has been reached, searching for another target", seconds=4.0)
                print("The target has been reached, searching for another target")
            

            th1 = Thread(target=agent._send_control_cmd)
            th2 = Thread(target=agent._send_driving_cmd)
            th3 = Thread(target=agent._get_vehicle_info_1)
            th4 = Thread(target=agent._get_vehicle_info_2)

            
            th1.start()
            th2.start()
            th3.start()
            th4.start()
            
            th1.join()
            th2.join()
            th3.join() 
            th4.join()

    finally:
        print("Exiting simulation..")
        if original_settings:
            sim_world.apply_settings(original_settings)
        
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
        '--rolename',
        metavar='NAME',
        default='sensors',
        help='actor role name (default: "sensors")')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Activate synchronous mode execution')
    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    try:
        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
        return

if __name__ == '__main__':
    main()
