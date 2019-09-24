#!/usr/bin/env python
#
# Temporary implementation of the interface with Arena.

# Guidance Node
import rospy
import paho.mqtt.client as mqtt
import signal
import time

from commander_interface.srv import GoTo 

from rarena import DroneArenaClass, TargetArenaClass 

scene = "/topic/luigi/"

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


# Signal handler for destroying object in Arena
def signal_handler(sig, frame):
        mqtt_client.publish(scene + "TargetGoto", "", retain=True)  
        mqtt_client.publish(scene + "floor", "", retain=True)  
        mqtt_client.publish(scene + "pad", "", retain=True)  
        mqtt_client.publish(scene + "cf2", "", retain=True)  


        time.sleep(1)

        mqtt_client.disconnect() #disconnect
        mqtt_client.loop_stop() #stop loop

        print("Removing objects before quiting...")
        time.sleep(1)	

signal.signal(signal.SIGINT, signal_handler)

if __name__ == '__main__':
    rospy.init_node('RArena_node')

    mqtt_client = mqtt.Client("client-ros", clean_session=True, userdata=None ) 
    mqtt_broker = "oz.andrew.cmu.edu"

    # Instatiate the MQTT client class
    print("Connecting to broker ", mqtt_broker)
    mqtt_client.connect(mqtt_broker)

    d1 = DroneArenaClass(toggle_active, mqtt_client, scene, 'cf1')
    d2 = DroneArenaClass(toggle_active, mqtt_client, scene, 'cf2')
 
    floor = TargetArenaClass(issue_command, mqtt_client, scene, 'floor',
            s =[3.0,0.005,3.0])
    
    land_pad = TargetArenaClass(land_command, mqtt_client, 
            scene, 'pad', c = "#AA00AA", 
            p = [1.0, 0.001, 1.0], s =[0.4,0.1,0.4])

    target_pad = TargetArenaClass(intercept_command, mqtt_client, 
            scene, 'target', c = "#3300AA", s = [0.4,0.1,0.4], isMovable=True)

    drones = dict()
    drones['cf1'] = d1
    drones['cf2'] = d2

    mqtt_client.loop_start() #start loop to process received mqtt messages

    rospy.spin()
