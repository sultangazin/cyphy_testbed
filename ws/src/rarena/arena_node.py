#!/usr/bin/env python
#
# Temporary implementation of the interface with Arena.

# Guidance Node
import rospy
import paho.mqtt.client as mqtt
import signal
import time
import numpy as np

from commander_interface.srv import GoTo 

from arena import DroneArenaClass, TargetArenaClass, NodeArenaClass, EdgeArenaClass

scene = "/topic/test"
entities = []

mqtt_client = mqtt.Client("client-ros", clean_session=True, userdata=None ) 
#mqtt_broker = "oz.andrew.cmu.edu"
mqtt_broker = "192.168.1.108"

# Services callbacks
goTo = rospy.ServiceProxy('/cf2/Commander_Node/goTo_srv', GoTo)

activeDrone = {'cf1': None, 'cf2': None}

## Callbacks for click events

# Click on drone
def toggle_active(name):
    for (k, v) in activeDrone.items():
        if (k != name):
            activeDrone[k] = None
        else:
            if (activeDrone[name]):
                activeDrone[name] = None
            else:
                activeDrone[name] = name

# Click on target
def issue_command(tg_p):
    if (activeDrone['cf1'] == None and activeDrone['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in activeDrone.items():
            if (v is not None):
                try:
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + 0.7], 3.0)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def land_command(tg_p):
    if (activeDrone['cf1'] == None and activeDrone['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in activeDrone.items():
            if (v is not None):
                try:
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + 0.7], 3.0)
                    time.sleep(4.0)
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2]], 3.0)

                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def intercept_command(p):
    print("Intercept requested")
    if (activeDrone['cf1'] == None and activeDrone['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in items(activeDrone):
            if (v is not None):
                try:
                    resp1 = drones[activeDrone].inTer(0.8, 7.0, 3.1)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))


# # Signal handler for destroying object in Arena
# def signal_handler(sig, frame):
#     # print("Removing objects before quitting...")
#     # mqtt_client.publish(scene + "/" + "cube_0", "", retain=True)  
#     # mqtt_client.publish(scene + "/" + "cube_1", "", retain=True) 
#     # mqtt_client.publish(scene + "/" + "cube_2", "", retain=True)
#     # mqtt_client.publish(scene + "/" + "line_0", "", retain=True)    
#     # mqtt_client.publish(scene + "/" + "line_1", "", retain=True)
#     # mqtt_client.publish(scene + "/" + "line_2", "", retain=True)

#     # time.sleep(1)

#     print("Quitting")
#     mqtt_client.disconnect() #disconnect
#     mqtt_client.loop_stop() #stop loop

# signal.signal(signal.SIGINT, signal_handler)

def generate_entities():
    global entities

    nodeA = NodeArenaClass(mqtt_client, scene, 'nodeA', id=0, 
      pos=[1,0.5,1], scale=[.2,.2,.2], pose_source="vrpn_client_node", color="#AA00AA")
    nodeB = NodeArenaClass(mqtt_client, scene, 'nodeB', id=1, 
      pos=[-1,0.5,-1], scale=[.2,.2,.2], pose_source="vrpn_client_node", color="#AA00AA")
    edge1 = EdgeArenaClass(mqtt_client, scene, 'edge1', id=2, 
      start_node=nodeA, end_node=nodeB, color="#AA00AA", animate=False)
    drone = DroneArenaClass(mqtt_client, scene, 'cf2', id=3, 
      scale=[.05,.02,.05], pose_source="vrpn_client_node", color="#0000AA")
    target = TargetArenaClass(mqtt_client, scene, 'target', id=4, 
      scale=[.3,.01,.3], pose_source="vrpn_client_node", color="#00AAAA")
    floor = TargetArenaClass(mqtt_client, scene, 'floor', id=5, 
      pos=[0,0,0], quat=[0,0,0,0], scale=[5,.01,3], color="#AAAAAA")
    workstation = NodeArenaClass(mqtt_client, scene, 'workstation', id=6, 
      pos=[-2.25,0.25,-1.4], scale=[.5,.5,.2], color="#AAAA00")
    edge2 = EdgeArenaClass(mqtt_client, scene, 'edge2', id=7, 
      start_node=workstation, end_node=drone, color="#AAAA00", animate=False)
      


    entities = [nodeA,nodeB,edge1,drone,target,floor,workstation,edge2]

def update_entities():
    global entities

    for entity in entities:
        entity.update()

def remove_entities():
    global entities
    print("Cleaning up all topics.")
    for entity in entities:
      print("Removing {}".format(entity))
      entity.remove()

    print("Quitting")
    mqtt_client.disconnect() #disconnect
    mqtt_client.loop_stop() #stop loop

if __name__ == '__main__':
    rospy.init_node('RArena_node')

    # # Instatiate the MQTT client class
    print("Connecting to broker ", mqtt_broker)
    mqtt_client.connect(mqtt_broker)

    # d1 = DroneArenaClass(toggle_active, mqtt_client, scene, 'cf1')
    # d2 = DroneArenaClass(toggle_active, mqtt_client, scene, 'cf2')
 
    # floor = TargetArenaClass(issue_command, mqtt_client, scene, 'floor',
    #         s =[3.0,0.005,3.0])
    
    # land_pad = TargetArenaClass(land_command, mqtt_client, w
    #         scene, 'pad', c = "#AA00AA", 
    #         p = [1.0, 0.001, 1.0], s =[0.4,0.1,0.4])

    # target_pad = TargetArenaClass(intercept_command, mqtt_client, 
    #         scene, 'target', c = "#3300AA", s = [0.4,0.1,0.4], isMovable=True)

    generate_entities()

    mqtt_client.loop_start() #start loop to process received mqtt messages

    #rospy.spin()
    rate = rospy.Rate(24)
    while not rospy.is_shutdown():
        update_entities()
        rate.sleep()

    rospy.on_shutdown(remove_entities)