# Manager Class

import rospy

from manager.srv import StartAll, StopAll, GetStatus, LandAll 
from commander_interface.srv import TakeOff, Land, Stop

class ManagerClass:

    def __init__(self):
        self.initData()

        self.loadParameters()

        self.registerServices()

        rospy.loginfo("\n [%s] Manager Initialized!"%rospy.get_name())

    def initData(self):
        self.land = dict()
        self.takeoff = dict()
        self.stop = dict() 
        pass

    def loadParameters(self):
        self.droneList = rospy.get_param('~droneList', ['cf2', 'cf3'])
        self.NumDrones = len(self.droneList) 
        pass

    def registerServices(self):
        # Advertise Services
        self.service_startAll = rospy.Service('startAll',
                StartAll, self.handle_startAll)

        self.service_landAll = rospy.Service('landAll',
                StopAll, self.handle_landAll)

        self.service_getStatus = rospy.Service('getStatus',
                GetStatus, self.handle_getStatus)

        self.service_stopAll = rospy.Service('stopAll',
                StopAll, self.handle_stopAll)

        # Subscribe to Services
        for ind, val in enumerate(self.droneList): 
            self.land[val] = rospy.ServiceProxy("/" + val + "/Commander_Node/land_srv", Land)
            self.takeoff[val] = rospy.ServiceProxy("/" + val + "/Commander_Node/takeoff_srv", TakeOff)
            self.stop[val] = rospy.ServiceProxy("/" + val + "/Commander_Node/stop_srv", Stop)

   
    def handle_startAll(self, req):
        h = 0.8
        if (req.h is not None):
            h = req.h

        for drone in self.droneList:
            self.takeoff[drone](h, 3.0)
        pass
        return True

    def handle_stopAll(self, req):
        for drone in self.droneList:
            self.stop[drone]()
        pass
        return True

    def handle_landAll(self, req):
        for drone in self.droneList:
            self.land[drone](3.0)

        return True

    def handle_getStatus(self, req):
        pass
        return True


