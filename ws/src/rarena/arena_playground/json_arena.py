#!/usr/bin/env python
import json
import numpy as np

# Helper functions for using Json messages


def parseEventJsonMsg(msg):
    jsonMsg=json.loads(msg.payload)
    # filter non-event messages
    if (jsonMsg["action"] !="clientEvent"): 
        return (False, 0, 0, 0, "");

    # filter non-mouse messages
    if (jsonMsg["type"]!="mousedown"):
        return (False, 0, 0, 0, "");

    click_x=jsonMsg['data']["position"]["x"]
    click_y=jsonMsg['data']["position"]["y"]
    click_z=jsonMsg['data']["position"]["z"]
    user = jsonMsg['data']["source"]
    
    return(True, click_x, click_y, click_z, user)

def parseCameraJsonMsg(msg):
    jsonMsg = json.loads(msg.payload)

    pos = np.zeros(3);
    quat = np.zeros(4);
    quat[0] = 1.0;

    try:
        msg_type = jsonMsg['type']
        obj_type = jsonMsg['data']['object_type']

        if ( (msg_type == "object") and (obj_type == "camera") ): 
            pos[0] = float(jsonMsg['data']["position"]["x"])
            pos[1] = float(jsonMsg['data']["position"]["y"])
            pos[2] = float(jsonMsg['data']["position"]["z"])

            quat[0] = float(jsonMsg['data']["rotation"]["w"])
            quat[1] = float(jsonMsg['data']["rotation"]["x"])
            quat[2] = float(jsonMsg['data']["rotation"]["y"])
            quat[3] = float(jsonMsg['data']["rotation"]["z"])
            return(True, pos, quat)
    
    except:
        print("Not camera message:")
        print(jsonMsg)
        print(" ")

    # If you reach here return False
    return (False, pos, quat)


def genListenerJsonMsg(shape, identifier, active):
    """
    Generate the Json message for enabling/disabling
    click-listener on an arena object.
    """
    if (active == True):
        action = "enable"
    else:
        action = "disable"

    json_message = {
                "object_id": shape + "_{}".format(identifier),
                "action": "update",
                "type": "object",
                "data": {
                    "click-listener":action
                    }
                }

    return json_message

def genTransJsonMsg(shape, identifier, opacity):
    json_message = {
            "object_id": shape + "_{}".format(identifier),
            "type":"object",
            "action": "update",
            "data": {
                "material": {"transparent": "true", "opacity": opacity}}
            }

    return json_message

def animationJsonMsg(shape, identifier, prop_, from_, to_, dur_):
    json_message = {
            "object_id": shape + "{}".format(identifier),
            "action": "update",
            "type": "object",
            "data": {
                "animation": { 
                    "property": prop_,
                    "from":from_,
                    "to": to_,
                    "loop": "true",
                    "dur": dur_}
                }
            }
    return json_message


def genDelJsonMsg(obj_id = ""):
    msg = {"object_id": obj_id, "action": "delete" }
    return msg


def genCameraJsonMsg(camera_id, pos, quat):
    json_message = {
                "object_id": camera_id,
                "action": "update",
                "type":"rig",
                "data": {
                    "position": {
                        "x": pos[0],
                        "y": pos[1],
                        "z": pos[2]
                        },
                    "rotation": {
                        "x": quat[0],
                        "y": quat[1],
                        "z": quat[2],
                        "w": quat[3]
                        }
                    }
                }
    return json_message

def genJsonLineMessage(id_, s, e, color="#AAAAAA"):
    json_message = {
            "object_id" : "line" + "_{}".format(id_),
            "action": "create",
            "data": {
                "object_type":
                "line",
                "start": {"x": s[0], "y": s[1], "z": s[2]},
                "end": {"x": e[0], "y": e[1], "z": e[2]}, 
                "color": color
                }
            }
    return json_message

def genJsonMessage(shape, identifier, action, pos=[0,0,0], quat=[1,0,0,0], scale=[1,1,1],
        color="", interactive=False, text=""): 
        if (action == 1):
            action = "create"
        else:
            action = "update"

        json_message = {
                "object_id": shape + "_{}".format(identifier),
                "action": action,
                "type":"object",
                "data": {
                    "object_type": shape,
                    "position": {
                        "x": pos[0],
                        "y": pos[1],
                        "z": pos[2]
                        },
                    "rotation": {
                        "x": quat[0],
                        "y": quat[1],
                        "z": quat[2],
                        "w": quat[3]
                        },
                    "scale": {
                        "x": scale[0],
                        "y": scale[1],
                        "z": scale[2]
                        },
                    "material": {}
                    }
                }

        if (color):
            if (shape == "text"):
                 json_message["data"]["color"] = color
            else:
                json_message["data"]["material"]["color"] = color

        if (interactive):
            json_message["data"]["click-listener"] = "enable"

        if (text):
            json_message["data"]["text"] = text

        return json_message
