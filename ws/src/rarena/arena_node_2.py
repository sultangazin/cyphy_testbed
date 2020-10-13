#!/usr/bin/env python3
#
# Temporary implementation of the interface with Arena.

# Guidance Node
import rospy
import paho.mqtt.client as mqtt
import signal
import time
import numpy as np
import yaml
import os 
import arena

from commander_interface.srv import GoTo

from rosarena2 import RosArenaObject, \
                      DroneArenaObject, \
                      SurfaceArenaObject, \
                      LinkArenaObject, \
                      TrajectoryArenaObject, statusFromMsg

from control_router.msg import NetworkStatusMsg

scene = "drone"
mqtt_broker = "arena.andrew.cmu.edu"
#mqtt_broker = "arena-west1.conix.io"

objects = []

drones = {}
nodes = {}
links = {}
        
status_topic = None

if status_topic:
    rospy.Subscriber(status_topic, NetworkStatusMsg, status_callback)
    rospy.loginfo("Subscribed to: {}".format(status_topic))

def status_callback(msg):
    active, id, freq = statusFromMsg(msg)
    if active:
        if id==3:
            nodes["nuc1"].activate()
            links["nuc1"].activate()
            nodes["nuc2"].deactivate()
            links["nuc2"].deactivate()
        if id==4:
            nodes["nuc2"].activate()
            links["nuc2"].activate()
            nodes["nuc1"].deactivate()
            links["nuc1"].deactivate()
    else:
        nodes["nuc1"].deactivate()
        links["nuc1"].deactivate()
        nodes["nuc2"].deactivate()
        links["nuc2"].deactivate()


def got_click(location):
    for name in drones:
        if drones[name].active == True:
            arena.Object(
            objName="line1",
            objType=arena.Shape.line,
            line=arena.Line(start=drones[name].location, end=location, color=(206, 0, 255)),
            ttl=2)
            break
    print("No drone selected")


def manage_nodes(objName):
    # Hack: Should be more procedural here, but relies on me indexing the links dict with nodes names.
    for name in nodes:
        nodes[name].deactivate()
        links[name].deactivate()
    nodes[objName].activate()
    links[objName].activate()


def generate_objects():
    drone = DroneArenaObject(objName="cf3", 
                             pose_source=None, 
                             location=(0,1,0), 
                             clickable=False, 
                             opacity=0.6)
    objects.append(drone)
    drones[drone.objName] = drone

    floor = SurfaceArenaObject(objName="floor", 
                               color=(100,150,100),
                               location=(0,0,0), 
                               scale=(4,0.02,3), 
                               opacity=0.6, 
                               clickable=False)
                               #ros_callback=got_click)
    objects.append(floor)

    camera = RosArenaObject(objName="camera", 
                          location=(2,2,-2), 
                          scale=(0.5,0.5,0.5), 
                          color=(50,50,50), 
                          clickable=False, 
                          opacity=1)
    objects.append(camera)

    nuc1 = RosArenaObject(objName="nuc1", 
                          location=(1,1,1), 
                          scale=(0.1,0.05,0.1), 
                          color=(200,0,200), 
                          clickable=False, 
                          opacity=0.7,
                          group_callback=None)
    objects.append(nuc1)
    nodes[nuc1.objName]=nuc1

    nuc2 = RosArenaObject(objName="nuc2", 
                          location=(-1,1,-1), 
                          scale=(0.1,0.05,0.1), 
                          color=(200,0,200), 
                          clickable=False, 
                          opacity=0.7,
                          group_callback=None)
    objects.append(nuc2)
    nodes[nuc2.objName]=nuc2

    link1 = LinkArenaObject(objName="link1", 
                            color=(200,0,0), 
                            objects=[drone, nuc1, camera])
    objects.append(link1)
    links[nuc1.objName]=link1

    link2 = LinkArenaObject(objName="link2", 
                            color=(200,0,0), 
                            objects=[drone, nuc2, camera])
    objects.append(link2)
    links[nuc2.objName]=link2

    # cow = arena.Object(
    # objName="model2",
    # objType=arena.Shape.gltf_model,
    # location=(-1, 1.8, -2),
    # scale=(0.02, 0.02, 0.02),
    # url="models/cow2/scene.gltf",
    # )

    # cow = arena.Object(
    # objName="model3",
    # objType=arena.Shape.gltf_model,
    # location=(-1, 1.8, -2),
    # scale=(0.02, 0.02, 0.02),
    # url="models/Cameras.gltf",
    # )
    # objects.append(cow)

def update_objects():
    for entity in objects:
        entity.arena_update()


def remove_objects():
    print("Cleaning up objects")
    for entity in objects:
      print("Removing {}".format(entity.objName))
      entity.delete()


if __name__ == '__main__':
    rospy.init_node('RArena_node')
    rospy.on_shutdown(remove_objects)

    # # Instatiate the MQTT client class
    print("Connecting to broker: ", mqtt_broker)
    arena.init(mqtt_broker, "realm", "drone")
    remove_objects()
    generate_objects()
    # mqtt_client.loop_start() #start loop to process received mqtt messages

    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Hack to deal with areana.handle_events blocking.
        while len(arena.messages) > 0:
            arena.process_message(arena.messages.pop(0))
        update_objects()
        rate.sleep()

