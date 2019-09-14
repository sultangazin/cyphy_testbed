#!/usr/bin/env python

# Guidance Node
import rospy
import paho.mqtt.client as mqtt
import signal
import time

from commander_interface.srv import GoTo 

from rarena import RArenaClass

scene = "/topic/luigi/"

# Services callbacks
goTo = rospy.ServiceProxy('/cf1/Commander_Node/goTo_srv', GoTo)


# Signal handler for destroying object in Arena
def signal_handler(sig, frame):
        mqtt_client.publish(scene + "TargetGoto", "", retain=True)  
        mqtt_client.publish(scene + "target", "", retain=True)  
        mqtt_client.publish(scene + "drone", "", retain=True)  


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


    c1 = RArenaClass(mqtt_client, scene, 'target', 'target', 
            scale=[3.0, 0.001, 3.0])
 
    c2 = RArenaClass(mqtt_client, scene, "drone", "drone",
            scale=[0.5, 0.1, 0.3])
   
    mqtt_client.loop_start() #start loop to process received mqtt messages

    rospy.spin()
