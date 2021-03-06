from classes import ROSArenaObject

class ROSArenaManager(object):
    def __init__(self, name):
        self.id = name + "_ROSArenaManager"
        self.objects = dict()


    def __del__(self):
        print("{} Destructor...\n".format(self.id))
        return
        

#    def add_object(self, name, ros_arena_obj):
#        """
#        Add an object to the Dictionary 
#        """
#        if name in self.objects:
#            #self.objects[name].data = ros_arena_obj.data
#            #return
#            pass
#        else:
#            self.objects[name] = ros_arena_obj
#
#        return


    def add_object(self, name, **kwargs):
        """
        Add an object to the Dictionary 
        """
        if name in self.objects:
            self.update_object(name, **kwargs)
        else:
            self.objects[name] = ROSArenaObject(**kwargs)

        return


    def delete_object(self, name):
        print("Deleting {}!\n".format(name))
        if name in self.objects:
            del self.objects[name]
        return


    def delete_objects(self):
        for (key, val) in self.objects.items():
            del val 
        return


    def update_object(self, name, **kwargs):
        if 'arena_srv' in kwargs: del kwargs['arena_srv']

        #self.objects[name].arena_obj_.update_attributes(**kwargs)
        if 'position' in kwargs:
            ros_pos = kwargs.get('position')
            self.objects[name].set_position(ros_pos)

        if 'rotation' in kwargs:
            ros_rot = kwargs.get('rotation')
            self.objects[name].set_rotation(ros_rot)

        if 'event_handler' in kwargs:
            handler = kwargs.get('event_handler')
            self.objects[name].arena_obj_.update_attributes(evt_handler = handler)

        if 'color' in kwargs:
            col = tuple(kwargs['color'])
            self.objects[name].set_color(col)

        if 'opacity' in kwargs:
            self.objects[name].set_opacity(kwargs.get('opacity'))
            
        if 'object_id' in kwargs and 'traj' in kwargs['object_id']:
            self.objects[name].arena_obj_.update_attributes(path = kwargs['path'])

        return



    def update(self):
        local_copy = self.objects.copy()
        items = local_copy.items()
        for (key, val) in items:
            val.update()

