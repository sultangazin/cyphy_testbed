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

from arena import DroneArenaClass, TargetArenaClass, NodeArenaClass, EdgeArenaClass, TrajectoryArenaClass, TrackerArenaClass

floor_offset = 0.9
realm_y_offset = 0

scene = "test"

entities = []

drones = {'cf3': None, 'cf2': None}

mqtt_client = mqtt.Client("client-ros", clean_session=True, userdata=None )
mqtt_broker = "oz.andrew.cmu.edu"
#mqtt_broker = "192.168.1.108:8080"


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
                    y = floor_offset
                    resp1 = drones[k].goTo([tg_p[0], tg_p[1], y], 3.0)
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



def generate_entities():
    global entities

    drone1 = DroneArenaClass(
            mqtt_client,
            scene,
            'cf3',
            id=3,
            source="vrpn_client_node",
            on_click_clb=toggle_active,
            pos=[0,0.05,-0.25],
            scale=[.1,.05,.1],
            color="#0044AA",
            text_visible = True,
            opacity=0.4)

    drone2 = DroneArenaClass(
            mqtt_client,
            scene,
            'cf2',
            id=4,
            source="vrpn_client_node",
            on_click_clb=toggle_active,
            pos=[0,0.05,0.25],
            scale=[.1,.05,.1],
            color="#8844AA",
            text_visible = True,
            opacity=0.4)

    drones['cf3'] = drone1
    drones['cf2'] = drone2

    target = TargetArenaClass(
            mqtt_client,
            scene,
            'target',
            id=5,
            source="vrpn_client_node",
            on_click_clb=intercept_command,
            color="#00AA88",
            scale=[0.3, 0.01, 0.3],
            opacity=0.4)

    floor = TargetArenaClass(mqtt_client,
            scene,
            'floor',
            id=6,
            on_click_clb=issue_command,
            color="#222222",
            pos=[0.0,realm_y_offset,0.5],
            quat=[0,0,0,1],
            scale=[4.5,.02,3],
            opacity=0.5,
            marker_offset=[0,floor_offset,0])

    land1 = TargetArenaClass(
            mqtt_client,
            scene,
            'land1',
            id=7,
            source="vrpn_client_node",
            on_click_clb=land_command,
            color="#0000AA",
            scale=[0.3, 0.01, 0.3],
            opacity=0.5)

    land2 = TargetArenaClass(
            mqtt_client,
            scene,
            'land2',
            id=8,
            source="vrpn_client_node",
            on_click_clb=land_command,
            color="#AA4400",
            scale=[0.3, 0.01, 0.3],
            opacity=0.5)

    nuc0 = NodeArenaClass(
            mqtt_client,
            scene,
            'nodeA',
            id=99,
            color="#AAAA00",
            scale=[0.1,0.03,0.1],
            opacity=0.5,
            source="vrpn_client_node")

    nuc1 = NodeArenaClass(
            mqtt_client,
            scene,
            'nodeB',
            id=98,
            color="#AAAA00",
            scale=[0.1,0.03,0.1],
            opacity=0.5,
            source="vrpn_client_node")

    edge1 = EdgeArenaClass(mqtt_client, scene, 'edge1', id=10,
       start_node=nuc0, end_node=drone1, color="#AAAA00", animate=True,
       packet_interval=1000, packet_duration=200, packet_scale=[.02,.02,.02])

    edge2 = EdgeArenaClass(mqtt_client, scene, 'edge2', id=11,
       start_node=nuc1, end_node=drone2, color="#AAAA00", animate=True,
       packet_interval=1000, packet_duration=200, packet_scale=[.02,.02,.02])

    edge3 = EdgeArenaClass(mqtt_client, scene, 'edge3', id=12,
       start_node=nuc0, end_node=nuc1, color="#00AA00", animate=True,
       packet_interval=1000, packet_duration=200, packet_scale=[.02,.02,.02])

    trajectory2 = TrajectoryArenaClass(mqtt_client, scene, 'trajectory2', id=13, source="cf2/mission_info",
      scale=[.02,.02,.02], opacity=0.5, tracked_object="vrpn_client_node/cf2/pose")

    trajectory3 = TrajectoryArenaClass(mqtt_client, scene, 'trajectory3', id=14, source="cf3/mission_info",
      scale=[.02,.02,.02], opacity=0.5, tracked_object="vrpn_client_node/cf3/pose")

#    center = NodeArenaClass(mqtt_client, scene, 'workstation', id=14,
#      color="#AAAAAA", pos=[0.07, realm_y_offset + 0.01, 0.1], scale=[0.3,0.02,0.3], opacity=0.7)

    

    # Add nodes for cameras

    ot1 = NodeArenaClass(mqtt_client, scene, 'ot1', id=15,
      color="#AA00AA", pos=[-3.0, realm_y_offset + 2.5, -2.0], scale=[0.15,0.15,0.15], opacity=0.5)

    ot2 = NodeArenaClass(mqtt_client, scene, 'ot2', id=16,
      color="#AA00AA", pos=[-1.0, realm_y_offset + 2.5, -3.0], scale=[0.15,0.15,0.15], opacity=0.5)

    ot3 = NodeArenaClass(mqtt_client, scene, 'ot3', id=17,
      color="#AA00AA", pos=[0.0, realm_y_offset + 2.5, -3.0], scale=[0.15,0.15,0.15], opacity=0.5)

    ot4 = NodeArenaClass(mqtt_client, scene, 'ot4', id=18,
      color="#AA00AA", pos=[2.0, realm_y_offset + 2.5, -3.0], scale=[0.15,0.15,0.15], opacity=0.5)

    ot5 = NodeArenaClass(mqtt_client, scene, 'ot5', id=19,
      color="#AA00AA", pos=[2.0, realm_y_offset + 2.5, 0.0], scale=[0.15,0.15,0.15], opacity=0.5)

    ot6 = NodeArenaClass(mqtt_client, scene, 'ot6', id=20,
      color="#AA00AA", pos=[2.0, realm_y_offset + 2.5, 2.5], scale=[0.15,0.15,0.15], opacity=0.5)

    ot7 = NodeArenaClass(mqtt_client, scene, 'ot7', id=21,
      color="#AA00AA", pos=[-0.5, realm_y_offset + 2.5, 2.5], scale=[0.15,0.15,0.15], opacity=0.5)

    ot8 = NodeArenaClass(mqtt_client, scene, 'ot8', id=22,
      color="#AA00AA", pos=[-3.0, realm_y_offset + 2.5, 2.5], scale=[0.15,0.15,0.15], opacity=0.5)


    # Initialize external trackers for evey viewing devices
#    nodeA_trk = TrackerArenaClass(mqtt_client, scene, "nodeA", "vrpn_client_node", active=True)
#    nodeB_trk = TrackerArenaClass(mqtt_client, scene, "nodeB", "vrpn_client_node", active=True)

    entities = [drone1,
                drone2,
                target,
                floor,
                land1,
                land2,
                nuc0,
                nuc1,
                edge1,
                edge2,
                edge3,
                trajectory2,
                trajectory3,
#                center,
                ot1,
                ot2,
                ot3,
                ot4,
                ot5,
                ot6,
                ot7,
                ot8,
#                nodeA_trk,
#                nodeB_trk
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
    


def configure_environment():
    # Set global attributes of arena environment
    #mqtt_client.publish(scene + "/Box-obj","",retain=True)
    pass


if __name__ == '__main__':
    rospy.init_node('RArena_node')

    # # Instatiate the MQTT client class
    print("Connecting to broker ", mqtt_broker)
    mqtt_client.connect(mqtt_broker)
    remove_conix_boxes()
    generate_entities()
    configure_environment()

    mqtt_client.loop_start() #start loop to process received mqtt messages

    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        update_entities()
        rate.sleep()

    rospy.on_shutdown(remove_entities)
