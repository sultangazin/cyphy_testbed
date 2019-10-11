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

from arena import DroneArenaClass, TargetArenaClass, NodeArenaClass, EdgeArenaClass, TrajectoryArenaClass

floor_offset = 0.7

scene = "/topic/test"
entities = []

drones = {'cf3': None, 'cf2': None}

mqtt_client = mqtt.Client("client-ros", clean_session=True, userdata=None ) 
mqtt_broker = "oz.andrew.cmu.edu"
#mqtt_broker = "192.168.1.108"


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
    if (drones['cf3'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    print("Issuing GOTO command to drone {}".format(k))
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + floor_offset], 3.0)
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def land_command(tg_p):
    if (drones['cf3'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    print("Issuing LAND command to drone {}".format(k))
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], tg_p[2] + floor_offset], 3.0)
                    time.sleep(4.0)
                    resp1 = drones[k].land(3.0)

                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

def intercept_command(p):
    print("Intercept requested")
    if (drones['cf3'] == None and drones['cf2'] == None):
        print("No drone selected!")
    else:
        for (k, v) in drones.items():
            if (v.isActive()):
                try:
                    resp1 = drones[k].inTer(1.5, 6.0, 3.5, 0.05)
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

    nodeA = NodeArenaClass(mqtt_client, scene, 'nodeA', id=0, source="vrpn_client_node",
      color="#7733AA", scale=[.2,.2,.2], opacity=0.3)

    nodeB = NodeArenaClass(mqtt_client, scene, 'nodeB', id=1, source="vrpn_client_node",
      color="#7733AA", scale=[.2,.2,.2], opacity=0.3)

    edge1 = EdgeArenaClass(mqtt_client, scene, 'edge1', id=2, 
      start_node=nodeA, end_node=nodeB, color="#AA00AA", ros_topic="vrpn_client_node/cf3/pose", 
      animate=True, interval=500, packet_duration=200)
      
    drone1 = DroneArenaClass(mqtt_client, scene, 'cf3', id=3, source="vrpn_client_node", on_click_clb=toggle_active,
      pos=[0,0.05,-0.25], scale=[.1,.05,.1], color="#0000AA", opacity=0.2)
      
    drone2 = DroneArenaClass(mqtt_client, scene, 'cf2', id=4, source="vrpn_client_node",  on_click_clb=toggle_active,
      pos=[0,0.05,0.25], scale=[.1,.05,.1], color="#A36044", opacity=0.2)
         
    drones['cf3'] = drone1
    drones['cf2'] = drone2

    target = TargetArenaClass(mqtt_client, scene, 'target', id=5, 
      color="#00AA3A", source="vrpn_client_node", on_click_clb=intercept_command, scale=[0.3, 0.05, 0.3], opacity=0.5)

    floor = TargetArenaClass(mqtt_client, scene, 'floor', id=6, 
      color="#222222", pos=[0,-0.21,0], quat=[0,0,0,0], scale=[5,.01,3], on_click_clb=issue_command, opacity=0.5)

    workstation = NodeArenaClass(mqtt_client, scene, 'workstation', id=7, 
      color="#AAAA00", pos=[-2.25,-0.21 + 0.25,-1.4], scale=[.5,.5,.2], opacity=0.5)

    edge2 = EdgeArenaClass(mqtt_client, scene, 'edge2', id=8, 
      start_node=workstation, end_node=drone2, color="#AAAA00", animate=False)

    land = TargetArenaClass(mqtt_client, scene, 'land', id=9, 
      color="#FF22EA", source="vrpn_client_node", on_click_clb=land_command, scale=[0.3, 0.05, 0.3], opacity=0.5)

    #traj = TrajectoryArenaClass(mqtt_client, scene, 'target', id=10, listen=True, scale=[.02,.02,.02])

      
    entities = [nodeA,
                nodeB,
                edge1,
                drone1,
                drone2,
                target,
                floor,
                workstation,
                edge2,
                land#,
                #traj
                ]

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

def remove_conix_boxes():
    mqtt_client.publish(scene + "/Box-obj","",retain=True)
    mqtt_client.publish(scene + "/fallBox","",retain=True)
    mqtt_client.publish(scene + "/fallBox2","",retain=True)

if __name__ == '__main__':
    rospy.init_node('RArena_node')

    # # Instatiate the MQTT client class
    print("Connecting to broker ", mqtt_broker)
    mqtt_client.connect(mqtt_broker)
    remove_conix_boxes()

    generate_entities()

    mqtt_client.loop_start() #start loop to process received mqtt messages

    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        update_entities()
        rate.sleep()

    rospy.on_shutdown(remove_entities)
