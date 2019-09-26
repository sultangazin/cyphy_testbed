#!/usr/bin/env python

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './')))

from commander_interface.srv import GoTo 
from guidance.srv import GenImpTrajectoryAuto

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
class RArenaClass(object):
    """
    Bridge class between Ros and Arena.
    """
    def __init__(self, mqtt_client, scene, name, shape="cube", id=0, 
        color="#AAAAAA", pos=[0,0,0], quat=[1,0,0,0], scale=[1,1,1]):

        self.client = mqtt_client
        self.scale = np.array([scale[0], scale[1], scale[2]])
        self.time_old = 0
        self.scene = scene
        self.name = name  
        self.color = color 
        self.pos = np.array(pos)
        self.quat = np.array(quat)

        #                                              x  y  z  qx qy qz qw sx sy sz col
        self.message = shape + "_{},".format(id) + "{},{},{},{},{},{},{},{},{},{},{},on"
        # self.obj_str = self.object_name + self.base_str 

        self.initArenaObject()
        rospy.loginfo("\n [%s] RArena Object Initialized!"%rospy.get_name())


    def initArenaObject(self):
        ############
        # Delete the object from the scene to get a fresh start with a null message
        self.client.publish(self.scene + self.name, "")  

        # Draw new object
        self.draw()

        # Enable click listener for object (allows it to be clickable)
        self.client.publish(self.scene + self.name + "/click-listener", 
                "enable", retain=True)

        #self.client.subscribe(self.scene_path + self.name + "/mouseup")
        self.client.subscribe(self.scene + self.name + "/mousedown")

        #self.client.message_callback_add(self.scene_path + self.name + "/mouseup", self.on_click_input)
        self.client.message_callback_add(self.scene + self.name + "/mousedown", 
                self.on_click_input)


    def __del__(self):
        self.client.publish(self.scene + self.name, "", retain=True)  

 
    def draw(self):
        # Fill mqtt message
        mqtt_string = self.message.format(
                str(self.pos[0]), str(self.pos[1]), str(self.pos[2]),
                str(self.quat[0]), str(self.quat[1]), str(self.quat[2]), str(self.quat[3]),
                self.scale[0], self.scale[1], self.scale[2], 
                self.color)

        # Draw object
        self.client.publish(self.scene + self.name, mqtt_string, retain=True)
        print("Updating: ", self.scene + self.name)
        print("Mqtt Command: ", mqtt_string)


    def update_pos(self, pos):
        self.pos[0] = float(pos[0])
        self.pos[1] = float(pos[1])
        self.pos[2] = float(pos[2])

    def update_quat(self, quat):
        self.quat[0] = float(quat[0])
        self.quat[1] = float(quat[1])
        self.quat[2] = float(quat[2])
        self.quat[3] = float(quat[3])


    def update_pose(self, pos, quat):
        self.pos[0] = float(pos[0])
        self.pos[1] = float(pos[1])
        self.pos[2] = float(pos[2])

        self.quat[0] = float(quat[0])
        self.quat[1] = float(quat[1])
        self.quat[2] = float(quat[2])
        self.quat[3] = float(quat[3])


    def on_click_input(self, client, userdata, msg):
        print("Got click: %s \"%s\"" % (msg.topic, msg.payload))
        click_x, click_y, click_z, user = msg.payload.split(',')
        print("Clicked by: " + user)    

    def set_color(self, new_color):
        self.color = new_color



###### NODE ARENA CLASS ####### 
class NodeArenaClass(RArenaClass):
    def __init__(self, mqtt_client, scene, name, id=0, color = "#AAAAAA",
            pos = [0,1,0], quat = [0,0,0,0], scale = [0.5, 0.5, 0.5], isMovable=False):

        super(NodeArenaClass, self).__init__(mqtt_client, scene, name=name, shape="cube", 
            id=id, color=color, pos=pos, quat=quat, scale=scale)

        if (isMovable):
            self.pose_topic_ =  "/" + self.name + "/external_pose"
            self.registerCallbacks()

        print("Created Arena Node Object")

    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        if str(msg.topic).find("mousedown") != -1:
            pass
                        
    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.pose_topic_)
        rospy.Subscriber(self.pose_topic_, PoseStamped, self.pose_callback)

    def pose_callback(self, pose_msg):
        self.pos = posFromPoseMsg(pose_msg)
        self.quat = quatFromPoseMsg(pose_msg)
       



###### DRONE ARENA CLASS ########
class DroneArenaClass(RArenaClass):
    # Constructor
    def __init__(self, on_click_clb, mqtt_clt, scene_path, name, isMovable = True):
        print("Creating Arena Drone Object")

        self.name = name
        self.isActive = False
        
        self.on_click = on_click_clb
 
        # Name of the topic containing the pose of the object       
        self.obj_pose_topic_ =  "/" + name + "/external_pose"

        self.position = [0,0.01,0]
        self.quaternion = [0,0,0,1]

        super(DroneArenaClass, self).__init__(mqtt_clt, scene_path, obj_name = self.name, 
               color="#0000FF", position=self.position, scale=[0.3,0.1,0.2])

        if (isMovable):
            self.registerCallbacks()

        self.registerServices()


    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.obj_pose_topic_)
        rospy.Subscriber(self.obj_pose_topic_, 
                PoseStamped, self.pose_callback)
       

    def pose_callback(self, pose_msg):
        self.position = posFromPoseMsg(pose_msg)
        self.quaternion = quatFromPoseMsg(pose_msg)
        
        # plot the object in arena
        super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion, color=self.color)

        
    def registerServices(self):
        #rospy.wait_for_service("/" + self.name + "/Commander_Node/goTo_srv")
        self.goTo = rospy.ServiceProxy("/" + self.name + "/Commander_Node/goTo_srv", GoTo)
        self.inTer = rospy.ServiceProxy("/" + self.name + "/gen_ImpTrajectoryAuto", GenImpTrajectoryAuto)
        pass


    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')
    
        if (self.isActive):
            self.isActive = False
            super(DroneArenaClass, self).set_color('#000022')
            self.color = "#000022"
            print("Drone {} Deselected\n".format(self.name))
            super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion)
        else:
            self.isActive = True
            super(DroneArenaClass, self).set_color('#00FF00')
            print("Drone {} Selected\n".format(self.name))
            super(DroneArenaClass, self).plot_arenaObj(self.position, self.quaternion)

        self.on_click(self.name) 
 

            
###### TARGET ARENA CLASS #######
class TargetArenaClass(RArenaClass):
    def __init__(self, on_click_clb, mqtt_client, scene, name, c = "#FFFFFF",
            p = [0,0,0], s = [0.5, 0.5, 0.5], isMovable=False):

        print("Creating Arena Target Object")
        self.on_click = on_click_clb
        super(TargetArenaClass, self).__init__(mqtt_client, scene, obj_name = name, 
                color = c, position = p, scale=s)

        if (isMovable):
            self.obj_pose_topic_ =  "/" + name + "/external_pose"
            self.registerCallbacks()
            
            
    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        # Create array with the target position

        if str(msg.topic).find("mousedown") != -1:
            print( "Target Position: " + str(click_x) + "," + str(click_y) + "," + str(click_z) )
            
            tg_string = "TargetGoto" + self.base_str;
            client.publish(self.scene_path + "TargetGoto", 
                    tg_string.format(click_x, click_y, click_z, 0.0, 0.0, 0.0, 0.0, 
                    0.1, 0.1, 0.1, "#FF0000"), retain=True) 

            tg_p = np.array([float(click_x), -float(click_z), float(click_y)])
            self.on_click(tg_p)
                        
    def registerCallbacks(self):
        # Subscribe to vehicle state update
        print("Subscribing to ", self.obj_pose_topic_)
        rospy.Subscriber(self.obj_pose_topic_, 
                PoseStamped, self.pose_callback)

    def pose_callback(self, pose_msg):
        self.position = posFromPoseMsg(pose_msg)
        self.quaternion = quatFromPoseMsg(pose_msg)
      
        # plot the object in arena
        super(TargetArenaClass, self).plot_arenaObj(self.position, self.quaternion, color=self.color)




###### EDGE ARENA CLASS #######
class EdgeArenaClass(RArenaClass):
    def __init__(self, on_click_clb, mqtt_client, scene, name, c = "#FFFFFF",
            p = [0,0,0], s = [0.5, 0.5, 0.5], isMovable=False, nodeA = None, nodeB = None):

        print("Creating Arena Edge Object")
        self.on_click = on_click_clb
        self.nodeA = nodeA
        self.nodeB = nodeB
        self.p = p
        self.scene = scene

        super(EdgeArenaClass, self).__init__(mqtt_client, scene, obj_name = name, 
                color = c, position = p, scale=s)

        # if (isMovable):
        #     self.obj_pose_topic_ =  "/" + name + "/external_pose"
        #     self.registerCallbacks()
        # else:
        #     self.obj_pose_topic = None

        tg_scene_string = self.scene_path + self.object_name + "/position"
        cmd_string = "x:{}; y:{}; z:{};".format(
            self.p[0], self.p[1], self.p[2])
        self.client.publish(tg_scene_string, cmd_string, retain=True)
        print("update position to: {}".format(cmd_string))

    def __del__(self):
            self.client.publish(self.scene + "edge", "", retain=True)  
            
    def on_click_input(self, client, userdata, msg):
        click_x, click_y, click_z, user = msg.payload.split(',')

        if str(msg.topic).find("mousedown") != -1:
            self.on_click(tg_p)

    # def registerCallbacks(self):
    #     pass

    def animate(self):
        # Trigger animation from nodeA position to nodeB position
        sc = self.scale
        cl = self.color

        tg_scene_string = self.scene_path + self.object_name + "/animation"
        cmd_string = "property: position; to: {} {} {}; duration: 1000; loop:true;".format(
            self.nodeB.p[0], self.nodeB.p[1], self.nodeB.p[2])
        self.client.publish(tg_scene_string, cmd_string)
        print("trigger animation")

        return
        
    def update(self):
        #tg_scene_string = self.scene_path + self.object_name +
    
        self.client.publish(self.scene + "edge", "line_1,{},{},{},{},{},{},0,0,0,0,#CE00FF,on".format( 
            self.nodeA.p[0],self.nodeA.p[1],self.nodeA.p[2],self.nodeB.p[0],self.nodeB.p[1],self.nodeB.p[2]), 
            retain=True)

        return



    # def plot_arenaObj(self, scale = None, color = None):
    #     """
    #     Update the coordinates of an object in arena and
    #     plot it in the synthetic environment
    #     """
    #     # Update the pos of the object
    #     self.p[0] = float(pos[0])
    #     self.p[1] = float(pos[1])
    #     self.p[2] = float(pos[2])

    #     if (scale is not None):
    #         sc = scale
    #     else:
    #         sc = self.scale

    #     if (color is not None):
    #         cl = color
    #     else:
    #         cl = self.color

    #     tg_scene_string = self.scene_path + self.object_name
    #     cmd_string = self.obj_str.format(
    #                 str(pos[0]), 
    #                 str(pos[2]), 
    #                 str(-pos[1]), 
    #                 str(quaternion[1]), str(quaternion[3]), str(-quaternion[2]), str(quaternion[0]), 
    #                 str(sc[0]), str(sc[1]), str(sc[2]), cl)

    #     time_now = time.time()
    #     diff = time_now - self.time_old
    #     if (diff) > 0.1:
    #         self.time_old = time_now 
    #         self.client.publish(tg_scene_string, 
    #                 cmd_string, 
    #                 retain=True)

    #     return
      
        # plot the object in arena
        # super(NodeArenaClass, self).plot_arenaObj(self.position, self.quaternion, color=self.color)

