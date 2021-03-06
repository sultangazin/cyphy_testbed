import numpy as np

from arena import *


class ROSArenaObject(object):
    """
    Class to bind ROS objects with ARENA objects.

    This is the interface between the two systems.
    I wanted to decouple the reference frames differneces and other details with this class.

    The ROSArenaObject "has" an Object.

    """
    def __init__(self, **kwargs):

        self.selected=False

        # Fetching information from KWARGS
        if 'arena_srv' in kwargs:
            self.arena_srv = kwargs.get('arena_srv')
            del kwargs['arena_srv']
        else:
            print("Must provide an Arena Server")

        if 'object_id' in kwargs:
            self.id = kwargs.get('object_id')
        else:
            print("Must provide an object ID")

        self.offset = kwargs.get('offset', np.array([0.0, 0.0, 0.0]))
        if 'offset' in kwargs: del kwargs['offset']

        self.clr = kwargs.get("color", [100, 100, 0])
        if "color" in kwargs: del kwargs["color"]

        self.opacity = kwargs.get("opacity", 1.0)
        if "opacity" in kwargs: del kwargs['opacity']

        self.ros_pos = kwargs.get('position', np.array([0, 0, 0], dtype = float)) 
        self.set_position(self.ros_pos)
        if "position" in kwargs:
            kwargs['position'] = self.arena_pos


        self.ros_rot = kwargs.get('rotation', np.array([1, 0, 0, 0], dtype=float)) 
        self.set_rotation(self.ros_rot)
        if "rotation" in kwargs: del kwargs['rotation']

        self.ros_scale = kwargs.get('scale', [1, 1, 1]); 
        self.set_arena_scale(self.ros_scale)
        kwargs['scale'] = self.arena_scale

        self.obj_type = kwargs.get('object_type')
        
        # Create the Arena Object
        mat_ = Material(
                color = tuple(self.clr),
                opacity = self.opacity)

        if (self.obj_type is not 'thickline'):
            kwargs['material'] = mat_
        else:
            kwargs['color'] = tuple(self.clr)

        self.arena_obj_ = Object(
                rotation = self.arena_rot,
                **kwargs)

        self.arena_srv.add_object(self.arena_obj_)

        self.update()
        self.updated = True;


    def __del__(self):
        #print("{} Destructor...".format(self.id))
        if self.arena_obj_ is not None:
            #print("{} Deleting...".format(self.id))
            self.arena_srv.delete_object(self.arena_obj_)
        #print("{} Destroyed\n".format(self.id))
        return


    def delete(self):
        #print("{} Deleting...\n".format(self.id))
        self.__del__()


    def set_position(self, ros_pos):
        """
        Set the position of the object converting from the 
        ros reference frame to the arena reference frame.
        """
        temp = np.array([
            ros_pos[0] + self.offset[0],
            ros_pos[2],
            -ros_pos[1] + self.offset[1]], dtype=float)

        self.arena_pos = Position(temp[0], temp[1], temp[2])

        self.updated = True;


    def set_rotation(self, ros_rot):
        """
        Set the rotation of the object converting from the 
        ros reference frame to the arena reference frame.
        """
        temp = np.array([
                    ros_rot[1],
                    ros_rot[3],
                    -ros_rot[2],
                    ros_rot[0]
                    ],
                dtype=float)

        self.arena_rot = Rotation(
                    temp[0],
                    temp[1],
                    temp[2],
                    temp[3],
                    )

        self.updated = True;

    def set_arena_scale(self, ros_scale):
        """
        Set the scale of the object converting from the 
        ros reference frame to the arena reference frame.
        """
        self.arena_scale = Scale(
                ros_scale[0], ros_scale[2], ros_scale[1])
        self.updated = True;


    def set_color(self, c):
        self.clr = tuple(c)
        self.updated = True


    def set_opacity(self, op):
        self.opacity = op


    def update(self):
        if (self.updated == True):
            mat = Material(
                color = tuple(self.clr),
                opacity = self.opacity)

            self.arena_obj_.update_attributes(
                position = self.arena_pos,
                rotation = self.arena_rot
                )

            if (self.obj_type is not "thickline"):
                self.arena_obj_.update_attributes(material=mat)
            else:
                self.arena_obj_.update_attributes(color=tuple(self.clr))

            self.arena_srv.update_object(self.arena_obj_)
            self.updated = False


    def get_arena_obj(self):
        return self.arena_obj_


    def get_id(self):
        return self.id
