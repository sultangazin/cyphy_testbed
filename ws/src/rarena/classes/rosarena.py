import rospy
import numpy as np

from arena import *


class ROSArenaObject:
    """
    Class to bind ROS objects with ARENA objects.
    """
    def __init__(self, **kwargs):

        self.selected=False
        self.id = kwargs.get('object_id')

        self.offset = np.array([-0.045, -0.15, 0]);

        self.arena_srv = kwargs.get('arena_srv')
        if 'arena_srv' in kwargs: del kwargs['arena_srv']

        p = kwargs.get('position', Position(0.0, 0.0, 0.0))
        q = kwargs.get('rotation', Rotation(0, 0, 0, 1));

        self.location = np.array(
                [p.x + self.offset[0], p.z, -p.y + self.offset[1]],
                dtype=float
                )

        self.rotation = np.array([q.x, q.z, -q.y, q.w], dtype=float)

        self.clr = kwargs.get("color", [100, 100, 0])
        if "color" in kwargs: del kwargs["color"]

        self.opacity = kwargs.get("opacity", 1.0)
        if "opacity" in kwargs: del kwargs['opacity']

        obj_type = kwargs.get('obj_type', "box")
        if "obj_type" in kwargs : del kwargs['obj_type']

        scale = kwargs.get('scale', [1,1,1]); 
        temp = scale[1]
        scale[1] = scale[2]
        scale[2] = temp
        if "scale" in kwargs: del kwargs['scale']

        # Create the Arena Object
        mat_ = Material(
                color = tuple(self.clr),
                opacity = self.opacity)

        self.arena_obj_ = Object(
                object_type = obj_type,
                material = mat_, 
                scale = scale,
                **kwargs)

        self.arena_srv.add_object(self.arena_obj_)

        self.updated = True;


    def __del__(self):
        print("{} Destructor...\n".format(self.id))
        if self.arena_obj_ is not None:
            self.arena_srv.delete_object(self.arena_obj_)


    def delete_obj(self):
        print("{} Deleting...\n".format(self.id))
        self.arena_srv.delete_object(self.arena_obj_)


    def set_position(self, p):
        self.location = p 
        self.updated = True;


    def set_color(self, c):
        self.clr = tuple(c)


    def set_opacity(self, op):
        self.opacity = op


    def set_rotation(self, r):
        self.updated = True;
        self.rotation = r 

    def arena_update(self):
        if (self.updated == True):
            p = Position(self.location[0], self.location[1], self.location[2])
            r = Rotation(self.rotation[0], self.rotation[1], self.rotation[2], self.rotation[3])

            mat = Material(color = tuple(self.clr), opacity = self.opacity)
            self.arena_obj_.update_attributes(
                    position = p,
                    rotation = r,
                    material = mat)

            self.arena_srv.update_object(self.arena_obj_)
            self.updated = False


    def get_arena_obj(self):
        return self.arena_obj_


    def get_id(self):
        return self.id

    def arena_callback(self, evt):
        pass 

    def register_sources(self):
        pass


    def state_callback(self, state_msg):
        pass


    def register_services(self):
        pass