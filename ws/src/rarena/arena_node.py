#!/usr/bin/env python
#
# Temporary implementation of the interface with Arena.

# Guidance Node
import rospy
import paho.mqtt.client as mqtt
import signal
import time
import numpy as np
import yaml
import os 

from commander_interface.srv import GoTo

from arena import DroneArenaClass, TargetArenaClass, NodeArenaClass, EdgeArenaClass, TrajectoryArenaClass, TrackerArenaClass, GeneralTrajectoryArenaClass

floor_offset = 0.9
realm_y_offset = 0

scene = "test"

entities = []

drones = {'cf3': None, 'cf2': None}
edges = {}

AnchorObject_list = {}
CameraObject_list = []
arena_nuc_list = []

AnchorEdges_list = []

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

# Toggle the network topology
flag = True
def toggle_network():
    global flag
    #edges["n0n1"].invertFlow();
    if (flag):
        edges["d0n1"].show();
        edges["d1n1"].show();
        edges["d0n0"].hide();
        edges["d1n0"].hide();
        flag = False
    else:
        edges["d0n1"].hide();
        edges["d1n1"].hide();
        edges["d0n0"].show();
        edges["d1n0"].show();
        flag = True

        
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
    id_cnt = 3;
    drone1 = DroneArenaClass(
            mqtt_client,
            scene,
            'cf3',
            id=id_cnt,
            source="vrpn_client_node/cf3",
            on_click_clb=toggle_active,
            pos=[0,0.05,-0.25],
            scale=[.1,.05,.1],
            color="#0044AA",
            text_visible = False,
            opacity=0.2)
    id_cnt = id_cnt + 1;

    drone2 = DroneArenaClass(
            mqtt_client,
            scene,
            'cf2',
            id=id_cnt,
            source="vrpn_client_node/cf2",
            on_click_clb=toggle_active,
            pos=[0,0.05,0.25],
            scale=[.01,.01,.01],
            color="#FFFFFF",
            text_visible = False,
            opacity=0.02)
    id_cnt = id_cnt + 1;

    drone2_est = DroneArenaClass(
            mqtt_client,
            scene,
            'cf2',
            id=id_cnt,
            source="cf2",
            pos=[0,0.05,0.25],
            scale=[.1,.05,.1],
            color="#8844AA",
            text_visible = False,
            opacity=0.2)
    id_cnt = id_cnt + 1;

    drones['cf3'] = drone1
    drones['cf2'] = drone2

    target = TargetArenaClass(
            mqtt_client,
            scene,
            'target',
            id=id_cnt,
            source="vrpn_client_node/target",
            on_click_clb=intercept_command,
            color="#00AA88",
            scale=[0.3, 0.01, 0.3],
            opacity=0.4)
    id_cnt = id_cnt + 1;
    
#    floor = TargetArenaClass(mqtt_client,
#            scene,
#            'floor',
#            id=id_cnt,
#            on_click_clb=issue_command,
#            color="#222222",
#            pos=[0.0,realm_y_offset,0.5],
#            quat=[0,0,0,1],
#            scale=[4.5,.02,3],
#            opacity=0.5,
#            marker_offset=[0,floor_offset,0])
#    id_cnt = id_cnt + 1;

    land1 = TargetArenaClass(
            mqtt_client,
            scene,
            'land1',
            id=id_cnt,
            source="vrpn_client_node/land1",
            on_click_clb=land_command,
            color="#AA4400",
            scale=[0.3, 0.01, 0.3],
            opacity=0.5) 
    id_cnt = id_cnt + 1;

    land2 = TargetArenaClass(
            mqtt_client,
            scene,
            'land2',
            id=id_cnt,
            source="vrpn_client_node/land2",
            on_click_clb=land_command,
            color="#0000AA",
            scale=[0.3, 0.01, 0.3],
            opacity=0.5)
    id_cnt = id_cnt + 1;

#    nuc0 = NodeArenaClass(
#            mqtt_client,
#            scene,
#            'nodeA',
#            id=id_cnt,
#            on_click_clb=toggle_network,
#            color="#AAAA00",
#            scale=[0.1,0.03,0.1],
#            opacity=0.5,
#            source="vrpn_client_node/nodeA")
#    id_cnt = id_cnt + 1;
#
#    nuc1 = NodeArenaClass(
#            mqtt_client,
#            scene,
#            'nodeB',
#            id=id_cnt,
#            on_click_clb=toggle_network,
#            color="#AAAA00",
#            scale=[0.1,0.03,0.1],
#            opacity=0.5,
#            source="vrpn_client_node/nodeB")
#    id_cnt = id_cnt + 1;

    # Control Edges 
    # These edges represent the data flow from the controllers to
    # the drones.
    # nuc0 -> drone0
#    edge_n0d0 = EdgeArenaClass(mqtt_client, scene, 'edge1', id=id_cnt,
#       start_node=nuc0, end_node=drone1, color="#AAAAAA", data_color="#FFFF00", 
#       animate=True, packet_interval=500, packet_duration=500, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["n0d0"] = edge_n0d0
#   # nuc1 -> drone1
#    edge_n1d1 = EdgeArenaClass(mqtt_client, scene, 'edge2', id=id_cnt,
#       start_node=nuc1, end_node=drone2, color="#AAAAAA", data_color="#FFFF00", 
#       animate=True, packet_interval=500, packet_duration=500, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#
#    # Sensor Edges
#    # These edges represent the information flow from the drones to the 
#    # control nodes.
#    # drone0 -> nuc0
#    edge_d0n0 = EdgeArenaClass(mqtt_client, scene, 'edge3', id=id_cnt,
#       start_node=drone1, end_node=nuc0, color="#FF7FFF", 
#       data_color="#FF7FFF", animate=True, packet_interval=500,
#       packet_duration=450, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["d0n0"] = edge_d0n0
#   # drone0 -> nuc1
#    edge_d0n1 = EdgeArenaClass(mqtt_client, scene, 'edge4', id=id_cnt,
#       start_node=drone1, end_node=nuc1, color="#FF7FFF",
#       data_color="#FF7FFF", animate=True, packet_interval=500,
#       packet_duration=450, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["d0n1"] = edge_d0n1
#
#    # drone1 -> nuc0
#    edge_d1n0 = EdgeArenaClass(mqtt_client, scene, 'edge5', id=id_cnt,
#       start_node=drone2, end_node=nuc0, color="#FF7FFF", 
#       data_color="#FF7FFF", animate=True, packet_interval=500,
#       packet_duration=450, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["d1n0"] = edge_d1n0
#
#   # drone1 -> nuc1
#    edge_d1n1 = EdgeArenaClass(mqtt_client, scene, 'edge6', id=id_cnt,
#       start_node=drone2, end_node=nuc1, color="#FF7FFF", animate=True,
#       data_color="#FF7FFF", packet_interval=500, packet_duration=450,
#       packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["d1n1"] = edge_d1n1



    # Inter Control Nodes
    # nuc0 -> nuc1
#    edge_n0n1 = EdgeArenaClass(mqtt_client, scene, 'edge7', id=id_cnt,
#       start_node=nuc0, end_node=nuc1, color="#00AA00", animate=True,
#       packet_interval=1000, packet_duration=500, packet_scale=[.02,.02,.02])
#    id_cnt = id_cnt + 1;
#    edges["n0n1"] = edge_n0n1


    # Trajectories nodes
    trajectory2 = TrajectoryArenaClass(mqtt_client, scene, 'trajectory2', id=id_cnt, source="cf2/mission_info",
      scale=[.02,.02,.02], opacity=0.5, tracked_object="vrpn_client_node/cf2/pose")
    id_cnt = id_cnt + 1;

    trajectory3 = TrajectoryArenaClass(mqtt_client, scene, 'trajectory3', id=id_cnt, source="cf3/mission_info",
      scale=[.02,.02,.02], opacity=0.5, tracked_object="vrpn_client_node/cf3/pose")
    id_cnt = id_cnt + 1;

    trajectory_simple = GeneralTrajectoryArenaClass(
            mqtt_client,
            scene,
            'trajectory_simple',
            id=id_cnt,
            source="cf2/ghost_trajectory",
            scale=[.005,.005,.005],
            opacity=0.1,
            tracked_object="vrpn_client_node/cf2/pose")
    id_cnt = id_cnt + 1;

#    center = NodeArenaClass(mqtt_client, scene, 'workstation', id=14,
#      color="#AAAAAA", pos=[0.07, realm_y_offset + 0.01, 0.1], scale=[0.3,0.02,0.3], opacity=0.7)

    
    dir_path = os.path.dirname(os.path.realpath(__file__))
    with open(dir_path + '/' + 'anchors.yaml') as anchor_file:
        # use safe_load instead load
        AnchorData = yaml.safe_load(anchor_file)
        
        numAnchors = AnchorData["NumOfAnchors"]
        print("Creating {} anchors objects...".format(numAnchors))

        for i in range(numAnchors):
            x = float(AnchorData[i]["x"])
            y = float(AnchorData[i]["y"])
            z = float(AnchorData[i]["z"])

            print("Adding Anchor Arena Object in " + 
                    "[{} {} {}]".format(x, y, z))

            AnchorObject_list['ao{}'.format(i)] = (
                    NodeArenaClass(mqtt_client,
                    scene,
                    'ao{}'.format(i),
                    id=id_cnt,
                    color="#00FF00",
                    pos=[x, realm_y_offset + z, -y],
                    scale=[0.15,0.15,0.15], opacity=0.5) 
            )
            id_cnt = id_cnt + 1;

    with open(dir_path + '/' + 'cameras.yaml') as camera_file:
        # use safe_load instead load
        CameraData = yaml.safe_load(camera_file)
        
        numCameras = CameraData["NumOfCameras"]
        print("Creating {} anchors objects...".format(numAnchors))

        for i in range(numCameras):
            x = float(CameraData[i]["x"])
            y = float(CameraData[i]["y"])
            z = float(CameraData[i]["z"])

            print("Adding Camera Arena Object in " + 
                    "[{} {} {}]".format(x, y, z))

            CameraObject_list.append(
                    NodeArenaClass(mqtt_client,
                        scene,
                        'ot{}'.format(i),
                        id=id_cnt,
                        color="#660066",
                        pos=[x, realm_y_offset + z, y],
                        scale=[0.15,0.15,0.15], opacity=0.5)
                    )
            id_cnt = id_cnt + 1;

    # Creating Edges Between Anchors and Drone
    for (k, v) in AnchorObject_list.items():
        AnchorEdges_list.append(
            EdgeArenaClass(
                mqtt_client,
                scene,
                'edge{}'.format(id_cnt),
                id=id_cnt,
                start_node=v,
                end_node=drone2,
                color="#333333",
                data_color="#FFFF00",
                animate=True,
                packet_interval=500,
                packet_duration=500,
                packet_scale=[.0,.0,.0]
            )
        )
        id_cnt = id_cnt + 1;
    

    # Initialize external trackers for evey viewing devices
    tablet_trk = TrackerArenaClass(mqtt_client, scene, "tablet", "vrpn_client_node", active=True)
#    nodeB_trk = TrackerArenaClass(mqtt_client, scene, "nodeB", "vrpn_client_node", active=True)

    entities = [drone1,
                drone2,
                drone2_est,
                target,
#                floor,
                land1,
                land2,
#                nuc0,
#                nuc1,
#                edge_n0d0,
#                edge_n1d1,
#                edge_d0n0,
#                edge_d0n1,
#                edge_d1n0,
#                edge_d1n1,
#                edge_n0n1,
                trajectory2,
                trajectory3,
                trajectory_simple,
#                center,
                tablet_trk,
#                nodeB_trk
                ]

    for (k, v) in AnchorObject_list.items():
        entities.append(v)

    for el in CameraObject_list:
        entities.append(el)

    for el in AnchorEdges_list:
        entities.append(el)

    ## Remove Nodes that are not currently in use... 
    for el in CameraObject_list:
        el.remove()

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
