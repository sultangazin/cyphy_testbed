
class PawnObject(object):
    """
    This class is the interface for ROS-Arena objects.
    This class still "has" a ROSArenaObject.
    """
    def get_arena_obj(self):
        pass
      
    def set_pose(self, ros_pos, ros_rot):
        pass
        
    def delete_obj(self, name):
        pass

    def update_event_handler(self, clback):
        pass

    def register_sources(self):
        pass

    def arena_update(self):
        pass

    def register_services(self):
        pass

    def state_callback(self, state_msg):
        pass

    def arena_callback(self, evt):
        pass
        
    
