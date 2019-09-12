#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import signal
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from commander_interface.srv import GoTo 

from geometry_msgs.msg import PoseStamped

import numpy as np


###### HELPERS #####
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.w,
        pose_msg.pose.orientation.x, 
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z])
    return quat 




################# ROS ARENA CLASS #####################
class RArenaClass:

    def __init__(self, scene, obj_name, obj_type, position=[0,0,0], scale=[1,1,1]):

        self.initData(scene, obj_name, obj_type, position, scale)

        self.loadParameters()

        if (obj_type == "drone"):
            self.registerCallbacks()

        if (obj_type == "target"):
            self.registerServices()

        self.initArena()

        rospy.loginfo("\n [%s] RArena Initialized!"%rospy.get_name()) 


    def initData(self, scene, obj_name, obj_type, pos, scale):
        self.scale = np.array([scale[0], scale[1], scale[2]])

        self.mqtt_broker = "oz.andrew.cmu.edu"
        self.scene_path = scene

        self.object_name = obj_name
        self.object_type = obj_type
        
        if (self.object_type == "drone"):
            print("Creating Arena Drone Object")
            self.color = "0000FF"
        else:
            print("Creating Arena Target Object")
            self.color = "FF0000"

        self.cube_str = self.object_name + ",{},{},{},0,0,0,0,{},{},{},{},on"

        self.p = np.array(pos)
        self.q = np.zeros(4)
        self.q[0] = 1.0

        self.has_datasource = False
 
        
    def loadParameters(self):
        if rospy.has_param('/' + self.object_name + '/external_pose'):
            self.has_datasource = True
            self.obj_pose_topic_ = rospy.get_param('topics/in_obj_pose_topic', '/' + self.object_name + '/external_pose')    


    def initArena(self):
        # Instatiate the MQTT client class
        self.client = mqtt.Client("client-" + self.object_name, clean_session=True, userdata=None ) 

        print("Connecting to broker ", self.mqtt_broker)
        self.client.connect(self.mqtt_broker) 

        ############
        # Setup box
        # Delete the object from the scene to get a fresh start with a null message
        self.client.publish(self.scene_path + self.object_name, "")  
        
        # Publish a cube with x,y,z and color parameters
        # retain=True makes it persistent
        self.client.publish(self.scene_path + self.object_name, 
            self.cube_str.format(self.p[0], self.p[1], self.p[2],
                self.scale[0], self.scale[1], self.scale[2], 
                self.color), 
            retain=True)

        # Enable click listener for object (allows it to be clickable)
        self.client.publish(self.scene_path + self.object_name + "/click-listener", "enable", retain=True)

        self.client.subscribe(self.scene_path + self.object_name + "/mouseup")
        self.client.subscribe(self.scene_path + self.object_name + "/mousedown")
        self.client.message_callback_add(self.scene_path + self.object_name + "/mouseup", self.on_click_input)
        self.client.message_callback_add(self.scene_path + self.object_name + "/mousedown", self.on_click_input)

        self.client.loop_start() #start loop to process received mqtt messages
        # add signal handler to remove objects on quit
        signal.signal(signal.SIGINT, self.signal_handler)


    def registerCallbacks(self):
        if (self.has_datasource):
            # Subscribe to vehicle state update
            rospy.Subscriber(self.obj_pose_topic_, 
                    CustOdometryStamped, self.odom_callback)

    def registerServices(self):
        rospy.wait_for_service('/cf1/Commander_Node/goTo_srv')
        self.goTo = rospy.ServiceProxy('/cf1/Commander_Node/goTo_srv', GoTo)
        

    ###### CALLBACKS
    # On new target information
    def odom_callback(self, pose_msg):

        pos = posFromPoseMsg(pose_msg)
        quat = quatFromPoseMsg(pose_msg)

        # Update the position of the object
        self.p[0] = pos[0]
        self.p[1] = pos[1]
        self.p[2] = pos[2]

        self.client.publish(self.scene_path + self.object_name, 
                self.cube_str.format(
                    self.p[0], 
                    self.p[1], 
                    self.p[2], 
                    self.scale[0], self.scale[1], self.scale[2],
                    self.color), 
                retain=False)

        return

    def signal_handler(self, sig, frame):
        self.client.publish(self.scene_path + self.object_name, "", retain=True)  
        print("Removing objects before I quit...")
        time.sleep(1)	
        sys.exit(0)


    def on_click_input(self, client, userdata, msg):

        if (self.object_type == "target"):
            print("got click: %s \"%s\"" % (msg.topic, msg.payload))
            click_x, click_y, click_z, user = msg.payload.split(',')
            print( "Clicked by: " + user )
            
            # Create array with the target position
            tg_p= np.array([float(click_x), float(click_y), float(click_z)])

            if str(msg.topic).find("mousedown") != -1:
                print( "Target Position: " + str(tg_p[0]) + "," + str(tg_p[1]) + "," + str(tg_p[2]) )

            try:
                resp1 = self.goTo([tg_p[0], tg_p[1], tg_p[2]], 2.0)
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))


        if (self.object_type == "drone"):
            print("Drone Selected\n")

