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

drones = {'cf1': None, 'cf2': None}

mqtt_client = mqtt.Client("client-ros", clean_session=True, userdata=None ) 
#mqtt_broker = "oz.andrew.cmu.edu"
mqtt_broker = "192.168.1.108"


## Callbacks for click events
# Click on drone
def toggle_active(name):
    for (k, v) in drones.items():
        if (k != name):
            if (v is not None):
                v.deactivate() 
        else:
            if (v.isActive()):
                v.deactivate() 
            else:
                v.activate()

# Click on target
def issue_command(tg_p):
    if (drones['cf1'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    print("Issuing GOTO command to drone {}".format(k))
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + 0.7], 3.0)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def land_command(tg_p):
    if (drones['cf1'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    print("Issuing LAND command to drone {}".format(k))
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + 0.7], 3.0)
                    time.sleep(4.0)
                    resp1 = drones[k].land(3.0)

                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def intercept_command(p):
    print("Intercept requested")
    if (drones['cf1'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    resp1 = drones[k].inTer(1.8, 5.0, 3.5, 0.1)
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
    
    drone1 = DroneArenaClass(mqtt_client, scene, 'cf1', id=3, 
      scale=[.05,.02,.05], pose_source="vrpn_client_node", color="#0000AA", on_click_clb=toggle_active)

    drone2 = DroneArenaClass(mqtt_client, scene, 'cf2', id=4, 
      scale=[.05,.02,.05], pose_source="vrpn_client_node", color="#00A0AA", on_click_clb=toggle_active)
   
    drones['cf1'] = drone1
    drones['cf2'] = drone2

    target = TargetArenaClass(mqtt_client, scene, 'target', id=5, 
      scale=[.3,.01,.3], pose_source="vrpn_client_node", color="#00AA3A", on_click_clb=intercept_command)

    floor = TargetArenaClass(mqtt_client, scene, 'floor', id=6, 
      pos=[0,0,0], quat=[0,0,0,0], scale=[5,.01,3], color="#AAAAAA", on_click_clb=issue_command)

    workstation = NodeArenaClass(mqtt_client, scene, 'workstation', id=7, 
      pos=[-2.25,0.25,-1.4], scale=[.5,.5,.2], color="#AAAA00")

    edge2 = EdgeArenaClass(mqtt_client, scene, 'edge2', id=8, 
      start_node=workstation, end_node=drone2, color="#AAAA00", animate=False)
    land = TargetArenaClass(mqtt_client, scene, 'land', id=9, 
      scale=[.3,.01,.3], pose_source="vrpn_client_node", color="#00EEEA", on_click_clb=land_command)

      
    entities = [nodeA,
            nodeB,
            edge1,
            drone1,
            drone2,
            target,
            floor,
            workstation,
            edge2,
            land]

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

    generate_entities()

    mqtt_client.loop_start() #start loop to process received mqtt messages

    #rospy.spin()
    rate = rospy.Rate(27)
    while not rospy.is_shutdown():
        update_entities()
        rate.sleep()

    rospy.on_shutdown(remove_entities)
