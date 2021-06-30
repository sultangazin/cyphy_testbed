# Simulated  Anchor Class 
import rospy
import numpy as np
import threading
import time

# Import the messages for the services
from anchor_sim.srv import AnchorSimCtrl, AnchorSimTeleCtrl 

# Import the messages for the topics
from anchor_sim.msg import AnchorData, AnchorSimStatus

# Import the message representing the Anchor Measurement from the crazyflie_driver package
from crazyflie_driver.msg import AnchorMeas

# Import the stamped pose message from the ROS standard geometry messages
from geometry_msgs.msg import PoseStamped

class Anchors(object):
    def __init__(self):
        # Initialziation and Node setup
        self.initData()
        self.loadParameters()
        self.initPubSub()
        self.registerServices()

        # Start thread
        self.thread = threading.Thread(target=self.status_thread)
        self.thread.start()
        rospy.loginfo("\n [%s] Anchor Simulator Initialized!"%rospy.get_name())


    def __del__(self):
        self.active_thread = False
        self.tele_thread.join()


    def initData(self):
        # Initialize the data

        # Dictionary to store the information about the anchors:
        # key: id of the anchor
        # value: dictionary with position of the anchor, flag for distortion and amount of distortion
        self.anchors = dict()
        self.NumAnchors = 0

        # Flag representing the activation state of the publishing thread
        self.active_thread = True
        self.status_freq = 1

        # Variable storing the position of the target in world frame
        self.target_pos = np.zeros(shape=(3, 1), dtype=float)

        # Variables storing the name of the topics
        self.pose_input_topic = ''
        self.sensors_output_topic = ''
        self.status_output_topic = ''

        # ROS publisher for the simulated sensor data 
        self.sensor_output_pub = None 
        # ROS publisher for the sensor status
        self.status_output_pub = None 

        # ROS services
        self.service_ctrl = None 
        self.service_status = None 

        # Thread publishing sensors status
        self.tele_thread = None

        # Mutex
        self.mx = threading.Lock()


    def loadParameters(self):
        # 1)
        # Topics name fetched from the Parameter Server
        # I am looking for a parameter in the private namespace of the node
        # i.e. /area0/sensors/anchors/topics/sensors_output_topic
        # https://wiki.ros.org/Parameter%20Server
        self.sensors_output_topic = rospy.get_param("~topics/sensors_output_topic")
        self.pose_input_topic = rospy.get_param("~topics/pose_input_topic")
        self.status_output_topic = rospy.get_param("~topics/status_output_topic")

        # Load Anchors from the yaml file that we specified in the launch file
        # https://riptutorial.com/ros/example/24423/launch-ros-nodes-and-load-parameters-from-yaml-file
        a_list = rospy.get_param('~anchors')
        self.NumAnchors = len(a_list)
        for i in range(self.NumAnchors):
            el = a_list[i]
            self.anchors[el['id']] = {
                        'pos': np.array([el['x'], el['y'], el['z']]),
                        'distorted': False,
                        'distortion': 0.0,
                        'meas': 0.0,
                        'enable': True
                        }

        rospy.loginfo("\n [{}] {} Anchors Loaded!".format(rospy.get_name(), self.NumAnchors))
        rospy.loginfo("\n [{}]: Output topic = {}".format(rospy.get_name(), self.sensors_output_topic))
        rospy.loginfo("\n [{}]: Input topic = {}".format(rospy.get_name(), self.pose_input_topic))


    def initPubSub(self):
        # Anchor Message Publisher
        nodens = rospy.get_name()
        self.sensor_output_pub = rospy.Publisher(
                nodens + "/" + self.sensors_output_topic, AnchorMeas, queue_size=3)
        # Global status Publisher 
        self.status_output_pub = rospy.Publisher(
                nodens + "/" + self.status_output_topic, AnchorSimStatus, queue_size=3)

        # Subscribe to the vehicle pose
        rospy.Subscriber(self.pose_input_topic, PoseStamped, self.poseCallback)


    def registerServices(self):
        # 3
        # Advertise Services

        # Service to control the behavior of the anchor simulation
        # The name of the service is given with the '~' so that it 
        # will be resolved relative to the node name: /<namespaces...>/<nodename>/<servicename>
        # i.e. /area0/sensors/anchors/anchorSimCtrl
        self.service_ctrl = rospy.Service('~anchorSimCtrl',
                AnchorSimCtrl, self.handle_ctrl_req)

        # Service to control the status streaming
        self.service_status = rospy.Service('~anchorSimStatus',
                AnchorSimTeleCtrl, self.handle_status_req)


    #=====================
    ###### CALLBACKS #####
    def poseCallback(self, pose_msg):
        # Update the position of the target
        self.target_pos[0] = pose_msg.pose.position.x;
        self.target_pos[1] = pose_msg.pose.position.y;
        self.target_pos[2] = pose_msg.pose.position.z;

        # Generate new measurements
        for anchor_id in range(self.NumAnchors):
            self.mx.acquire()
            anchor_data = self.anchors[anchor_id]

            # Compute the distance
            anchor_pos = anchor_data['pos']
            anchor_meas = np.linalg.norm(self.target_pos - anchor_pos)
            # Add distortion if distorted
            if anchor_data['distorted']:
                anchor_meas = anchor_meas + anchor_data['distortion']
            anchor_data['meas'] =  anchor_meas
            self.mx.release()


    def status_thread(self):
        while (self.active_thread and not rospy.is_shutdown()):
            r = rospy.Rate(self.status_freq);

            msg = AnchorSimStatus()
            msg.header.stamp = rospy.Time.now()
            msg.anchors = []

            for (index, data) in self.anchors.items():
                self.mx.acquire()
                m = AnchorData()
                m.id = index 
                m.pos = data['pos']
                m.meas = data['meas']
                m.distortion = data['distortion']
                m.isDistorted = data['distorted']
                msg.anchors.append(m)

                # Compose the ROS message and publish it if anchor enabled
                if data['enable']:
                    anch_msg = AnchorMeas()
                    anch_msg.dist = m.meas
                    anch_msg.id = m.id 
                    anch_msg.x_anchor = m.pos[0]
                    anch_msg.y_anchor = m.pos[1]
                    anch_msg.z_anchor = m.pos[2]

                    # Publish the message
                    self.sensor_output_pub.publish(anch_msg)
                    time.sleep(0.0001)
                self.mx.release()

            # Publish the network status
            self.status_output_pub.publish(msg)
            r.sleep()

   
    def handle_ctrl_req(self, req):
        # This callback is associated with the service that controls the behavior of the anchors.
        # In the Service Message the request part containts the settings that the user
        # wants to change. We assume that the user can request the changes for a specific anchor, so
        # we have an ID and then the settigs.
        # The reply part we just return a boolean.
        anchor_id = req.id
        self.anchors[anchor_id]['enable'] = req.enable 
        self.anchors[anchor_id]['distorted'] = req.enable_distortion 
        self.anchors[anchor_id]['distortion'] = req.distortion 
        return True


   
    def handle_status_req(self, req):
        self.status_freq = max(req.freq, 0)
        if req.active:
            if self.active_thread == False:
                self.active_thread = True
                self.thread = threading.Thread(target=self.status_thread)
                self.thread.start()
        else:
            self.active_thread = False

        return True
