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

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../guidance/trjgen')))

from commander_interface.srv import GoTo, Land
from guidance.srv import GenImpTrajectoryAuto

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3

from testbed_msgs.msg import MissionMsg
from trjgen.class_bz import Bezier

import numpy as np

y_offset=0

###### HELPERS #####
def posFromPoseMsg(pose_msg):
    pos = np.array([pose_msg.pose.position.x, 
                pose_msg.pose.position.y + y_offset, 
                pose_msg.pose.position.z])
    return pos

def quatFromPoseMsg(pose_msg):
    quat = np.array([pose_msg.pose.orientation.x, 
                     pose_msg.pose.orientation.y,
                     pose_msg.pose.orientation.z,
                     pose_msg.pose.orientation.w])
    return quat 

# Convert from quaternion to euler angles
def ToEulerAngles(q):
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q[0] * q[1] + q[2] * q[3]);
    cosr_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
    roll = math.atan2(sinr_cosp, cosr_cosp);

    # pitch (y-axis rotation)
    sinp = 2.0 * (q[0] * q[2] - q[3] * q[1]);

    pitch = math.asin(sinp);

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2]);
    cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]);

    yaw = math.atan2(siny_cosp, cosy_cosp);

    return np.array([roll, pitch, yaw])

def Rx(a):
    # Roll Matrix
    R = np.eye(3)
    R[1][1] = math.cos(a)
    R[1][2] = -math.sin(a)
    R[2][1] = math.sin(a)
    R[2][2] = math.cos(a)
    return R
    
def Ry(a):
    # Pitch Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][2] = math.sin(a)
    R[2][0] = -math.sin(a)
    R[2][2] = math.cos(a)
    return R

def Rz(a):
    # Yaw Matrix
    R = np.eye(3)
    R[0][0] = math.cos(a)
    R[0][1] = -math.sin(a)
    R[1][0] = math.sin(a)
    R[1][1] = math.cos(a)
    return R


def quat2Rot(q):
    # Quaternion to Rotation matrix
    R = np.zeros((3,3))
    eul = ToEulerAngles(q)

    # Yaw * Pitch * Roll (Robotic convention)
    R = np.matmul(np.matmul(Rz(eul[2]), Ry(eul[1])), Rx(eul[0]))
    return R


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
                 clickable=False, 
                 source=None,
                 ros_callback=None,
                 group_callback=None):

        self.location=location
        self.rotation=None
        self.baseColor=color
        self.activeColor=activeColor
        self.hoverColor=hoverColor
        self.opacity=opacity
        self.source=source
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
                         clickable=clickable,
                         callback=self.arena_callback)

        self.update(transparency=arena.Transparency(True, self.opacity))

        self.register_source()
        self.register_services()

    def register_source(self):
        if self.source:
            # Subscribe to some pose topic
            rospy.Subscriber(self.source, PoseStamped, self.source_callback)
            rospy.loginfo("Subscribed to: {}".format(self.source))

    def register_services(self):
        pass

    def source_callback(self, pose_msg):
        # Update pose information
        self.location = posFromPoseMsg(pose_msg)
        self.rotation = quatFromPoseMsg(pose_msg)

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
            if msg.event_action != arena.arena.EventAction.clientEvent:
                return

            if msg.event_type  == arena.arena.EventType.mouseenter:
                self.hover=True
                    
            elif msg.event_type == arena.arena.EventType.mouseleave:
                self.hover=False

            elif msg.event_type  == arena.arena.EventType.mouseup:
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
                 clickable=True, 
                 source=None,
                 ros_callback=None):

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         clickable=clickable,
                         source=source,
                         ros_callback=ros_callback)


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
                 source=None,
                 ros_callback=None):

        self.offset = (0,0,0)

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         clickable=clickable,
                         source=source,
                         ros_callback=ros_callback)

    def arena_update(self):
        if self.hover:
            color=self.hoverColor
        else:
            color = self.baseColor
        self.update(location=self.location, rotation=self.rotation, color=color)

    def arena_callback(self, msg):
        if msg.event_action != arena.arena.EventAction.clientEvent:
            return

        if msg.event_type  == arena.arena.EventType.mouseenter:
            self.hover=True

        elif msg.event_type == arena.arena.EventType.mouseleave:
            self.hover=False

        elif msg.event_type  == arena.arena.EventType.mouseup:
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
                 objects = None,
                #  opacity=1,
                 source=None,
                 ros_callback=None):

        self.line_width=line_width
        self.objects=objects
        self.source=source
        self.baseColor=color
        self.activeColor=activeColor
        self.ros_callback=ros_callback
        self.active=False

        path = self.get_path() if self.objects else [(0,1,1),(0,1,-1)]

        super().__init__(objName=objName, 
                         objType=objType, 
                         color=color, 
                         thickline=arena.Thickline(line_width=line_width, 
                                                   color=color, 
                                                   path=path))
        # self.update(transparency=arena.Transparency(True, opacity))

        self.register_source()

    def get_path(self):
        path = [obj.location for obj in self.objects]
        return path

    def register_source(self):
        if self.source:
            # Subscribe to some pose topic
            rospy.Subscriber(self.source, PoseStamped, self.source_callback)
            rospy.loginfo("Subscribed to: {}".format(self.source))

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
                 source=None,
                 ros_callback=None):

        self.offset = (0,0,0)

        super().__init__(objName=objName, 
                         objType=objType, 
                         location=location, 
                         rotation=rotation,
                         scale=scale,
                         color=color,
                         opacity=opacity,
                         clickable=clickable,
                         source=source,
                         ros_callback=ros_callback)