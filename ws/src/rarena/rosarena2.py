#!/usr/bin/env python3

import paho.mqtt.client as mqtt
import rospy
import os
import sys
import time
import json
import math
import arena

from arena_playground.json_arena import *

# sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../guidance/trjgen')))

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from control_router.msg import NetworkStatusMsg

import numpy as np

y_offset=0

###### HELPERS #####
def posFromPoseMsg(pose_msg):
    # Z-up to Y-up
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.z + y_offset, 
                pose_msg.pose.position.y])
    return pos

def quatFromPoseMsg(pose_msg):
    # Z-up to Y-up
    quat = np.array([pose_msg.pose.orientation.x, 
                     -pose_msg.pose.orientation.z,
                     -pose_msg.pose.orientation.y,
                     -pose_msg.pose.orientation.w])
    return quat 

def statusFromMsg(status_msg):
    active = status_msg.network_ctrl_active
    id = status_msg.active_controller_id
    freq = 10
    return active, id, freq


################# ROS ARENA CLASS #####################
class RosArenaObject(arena.Object):
    def __init__(self, 
                 objName="cube", 
                 objType=arena.Shape.cube, 
                 location=(0,0,0),
                 rotation=(0,0,0,0),
                 scale=(1,1,1),
                 color=(200,200,200),
                 hoverColor=(0,200,0),
                 activeColor=(200,200,0),
                 opacity=1,
                 url=None,
                 pose_source=None,
                 active_source=None,
                 ros_callback=None,
                 group_callback=None,
                 clickable=False):

        self.location=location
        self.rotation=rotation
        self.baseColor=color
        self.hoverColor=hoverColor
        self.activeColor=activeColor
        self.opacity=opacity
        self.pose_source=pose_source
        self.active_source=active_source
        self.ros_callback=ros_callback
        self.group_callback=group_callback
        self.hover=False
        self.active=False

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         url=url,
                         ttl=2,
                         clickable=clickable,
                         callback=self.arena_callback)

        self.update(transparency=arena.Transparency(True, self.opacity))

        self.register_sources()
        self.register_services()

    def register_sources(self):
        if self.pose_source:
            # Subscribe to some pose topic
            rospy.Subscriber(self.pose_source, PoseStamped, self.pose_callback)
            rospy.loginfo("Subscribed to: {}".format(self.pose_source))
        if self.active_source:
            pass
            #register source        

    def register_services(self):
        pass

    def pose_callback(self, pose_msg):
        # Update pose information
        self.location = posFromPoseMsg(pose_msg)
        self.rotation = quatFromPoseMsg(pose_msg)

    def active_callback(self, active_msg):
        pass
        # activate/deactivate

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def arena_update(self):
        if self.active:
            color=self.activeColor
        elif self.hover:
            color=self.hoverColor
        else:
            color = self.baseColor
        self.update(location=self.location, rotation=self.rotation, color=color)
        # self.update(rotation=self.rotation)

    def arena_callback(self, msg):
        # active_opacity=0.5
        if self.clickable:
            if msg.event_action != arena.EventAction.clientEvent:
                return

            if msg.event_type  == arena.EventType.mouseenter:
                self.hover=True
                    
            elif msg.event_type == arena.EventType.mouseleave:
                self.hover=False

            elif msg.event_type  == arena.EventType.mouseup:
                if self.active:
                    self.active = False
                    print("deactivate ({})".format(self.objName))
                else:
                    self.active = True
                    print("activate ({})".format(self.objName))
                    if self.group_callback:
                        self.group_callback(self.objName)
                    if self.ros_callback:
                        self.ros_callback()


class DroneArenaObject(RosArenaObject):
    def __init__(self, 
                 objName="drone", 
                 objType=arena.Shape.sphere, 
                 location=(0,0,0),
                 rotation=(0,0,0,0),
                 scale=(.1,.1,.1),
                 color=(200,200,200),
                 opacity=1,
                 url=None,
                 clickable=True, 
                 pose_source=None,
                 active_source=None,
                 ros_callback=None,
                 group_callback=None):


        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         url=url,
                         clickable=clickable,
                         pose_source=pose_source,
                         active_source=active_source,
                         ros_callback=ros_callback,
                         group_callback=group_callback)


class SurfaceArenaObject(RosArenaObject):
    def __init__(self, 
                 objName="surface", 
                 objType=arena.Shape.cube, 
                 location=(0,0,0),
                 rotation=(0,0,0,0),
                 scale=(.1,.1,.1),
                 color=(200,200,200),
                 opacity=1,
                 clickable=True, 
                 pose_source=None,
                 active_source=None,
                 ros_callback=None,
                 group_callback=None):

        self.offset = (0,0,0)

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         clickable=clickable,
                         pose_source=pose_source,
                         active_source=active_source,
                         ros_callback=ros_callback,
                         group_callback=group_callback)

    def arena_update(self):
        if self.hover:
            color=self.hoverColor
        else:
            color = self.baseColor
        self.update(location=self.location, rotation=self.rotation, color=color)

    def arena_callback(self, msg):
        if msg.event_action != arena.EventAction.clientEvent:
            return

        if msg.event_type  == arena.EventType.mouseenter:
            self.hover=True

        elif msg.event_type == arena.EventType.mouseleave:
            self.hover=False

        elif msg.event_type  == arena.EventType.mouseup:
            location = tuple(float(num) for num in msg.position)
            arena.Object(location=location + self.offset, 
                         objType=arena.Shape.cylinder, 
                         scale=(.1,.1,.1), 
                         color=(200,0,0), 
                         ttl=1)
            if self.ros_callback:
                self.ros_callback(location)

class LinkArenaObject(arena.Object):
    def __init__(self, 
                 objName="link", 
                 objType=arena.Shape.thickline, 
                 line_width=5,
                 color=(200,0,0),
                 activeColor=(0,200,200),
                 objects = None
                #  opacity=1,
                 ):

        self.line_width=line_width
        self.baseColor=color
        self.activeColor=activeColor
        self.objects=objects
        self.active=False

        path = self.get_path() if self.objects else [(0,1,1),(0,1,-1)]

        super().__init__(objName=objName, 
                         objType=objType, 
                         color=color, 
                         thickline=arena.Thickline(line_width=line_width, 
                                                   color=color, 
                                                   path=path))
        # self.update(transparency=arena.Transparency(True, opacity))

        # self.register_source()

    def get_path(self):
        path = [obj.location for obj in self.objects]
        return path

    # def register_source(self):
        # if self.pose_source:
        #     # Subscribe to some pose topic
        #     rospy.Subscriber(self.pose_source, PoseStamped, self.source_callback)
        #     rospy.loginfo("Subscribed to: {}".format(self.pose_source))

    def source_callback(self, pose_msg):
        pass

    def activate(self):
        self.active = True

    def deactivate(self):
        self.active = False

    def arena_update(self):
        if self.active:
            self.thickline=arena.Thickline(line_width=self.line_width, 
                                                   color=self.activeColor, 
                                                   path=self.get_path())
            self.update()
        else:
            self.thickline=arena.Thickline(line_width=1, 
                                                   color=self.baseColor, 
                                                   path=self.get_path())
            self.update()
                                                
        # self.update(path=self.location)


class TrajectoryArenaObject(RosArenaObject):
    def __init__(self, 
                 objName="trajectory", 
                 objType=arena.Shape.cube, 
                 location=(0,0,0),
                 rotation=(0,0,0,0),
                 scale=(.1,.1,.1),
                 color=(200,200,200),
                 opacity=1,
                 clickable=True, 
                 pose_source=None,
                 active_source=None,
                 ros_callback=None,
                 group_callback=None):

        self.offset = (0,0,0)

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         clickable=clickable,
                         pose_source=pose_source,
                         active_source=active_source,
                         ros_callback=ros_callback,
                         group_callback=group_callback)