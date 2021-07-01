#!/bin/python

## IMPORTS
import sys
import os
from signal import signal, SIGINT
from sys import exit
import numpy as np
import json
import os
import time

import rospy
import rospkg

from arena import *

from classes import ROSArenaObject
from classes import DroneObject 

## GLOBAL
host = "arenaxr.org"
realm = "realm"
scene = os.environ['SCENE']
#scene='LandOfOz'

# Arena Main object 
# The scene should be specified with the environmental variable SCENE
#arena_scene = Scene(host=host, realm=realm, scene=os.environ['SCENE'])
arena_scene = Scene(host=host, realm=realm, scene=scene)

# List of arena entities
arena_entities = dict()


def callback_monitor(mgs):
    ent = msg.entity
    active = msg.active
    if active:
        arena_entities[ent].set_color((255, 255, 0))
        #arena_entities['cf2'].set_opacity(0.7)
    else:
        arena_entities[ent].set_color((0, 255, 0))
        #arena_entities['cf2'].set_opacity(0.0)


def handler(signal_received, frame):
    global arena_scene
    global arena_entities

    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')

    arena_scene.stop_tasks()

    for (key, value) in arena_entities.items():
        value.delete()

    print('TERMINATED')

    exit(0)


def load_entities():
    global arena_entities
    global arean_scene
    rospack = rospkg.RosPack()

    atlas_data = open(rospack.get_path('rarena') + '/config/atlas.json', 'r')

    # Load information from the configuration file
    json_data = json.load(atlas_data)
    print("Loading Objects...\n")
    for el in json_data['entities']:
        pos = el["pos0"];
        entity = None
        if (el["type"] == "object"):
            print("Found {} with name {} @ {}\n".format(
                el['type'], el['name'], el['pos0']))
            entity = ROSArenaObject(
                    arena_srv = arena_scene,
                    object_id = el["name"],
                    object_type = el["shape"],
                    scale = el["scale"],
                    color = el["color"],
                    opacity = el["opacity"],
                    persist = True,
                    position = np.array(
                        [pos[0], pos[1], pos[2]],
                        dtype=float))


        if (el["type"] == "pawn"):
            print("Found {} with name {} @ {}\n".format(
                el['type'], el['name'], el['pos0']))
            entity = DroneObject(
                    arena_srv = arena_scene,
                    object_id = el["name"],
                    object_type = el["shape"],
                    scale = el["scale"],
                    color = el["color"],
                    opacity = el["opacity"],
                    position = np.array(
                        [pos[0], pos[1], pos[2]],
                        dtype=float))

        if (entity is not None):
            arena_entities[el["name"]] = entity 
    print("Loading Objects: Done!\n")


@arena_scene.run_once
def setup():
    signal(SIGINT, handler)

    # Load Objects from file.
    load_entities()


@arena_scene.run_forever(interval_ms=100)
def periodic():
    # Update the entities of the arena
    for key, entity in arena_entities.items():
        # Update the Object
        entity.update()
    

if __name__ == '__main__':
    rospy.init_node('RArena_node')
    # rospy.on_shutdown(remove_objects)

    # Subscribe to topics produced by the ROS framework
    #rospy.Subscriber("/CISSupervisor/cf2/cis_perf", PerformanceMsg, callback_monitor)

    # Start Arena tasks
    arena_scene.run_tasks()

    rospy.spin()
    
    


