# Simulated  Anchor Class 
import rospy
import numpy as np
import threading

# Import the messages for the services
from anchor_sim.srv import AnchorSimCtrl, AnchorSimTeleCtrl 
from anchor_sim.msg import AnchorData, AnchorSimStatus

# Import the message representing the Anchor Measurement from the crazyflie_driver package
from crazyflie_driver.msg import AnchorMeas

# Import the stamped pose message from the ROS standard geometry messages
from geometry_msgs.msg import PoseStamped

class Anchors(object):
    def __init__(self):
        self.initData()
        self.loadParameters()
        self.registerServices()
        self.initPubSub()

        # Start thread
        self.thread = threading.Thread(target=self.status_thread)
        self.thread.start()
        rospy.loginfo("\n [%s] Anchor Simulator Initialized!"%rospy.get_name())

    def __del__(self):
        self.active_thread = False
        self.tele_thread.join()

    def initData(self):
        self.anchors = dict()
        self.group_meas = dict() 
        self.NumAnchors = 0

        self.active_thread = True
        self.status_freq = 1

        self.target_pos = np.zeros(shape = (3, 1), dtype=float)

        self.sensor_output_topic = ''
        self.pose_input_topic = ''
        self.status_output_topic = ''

        self.sensor_output_pub = None 

        self.service_ctrl = None 
        self.service_status = None 

        self.tele_thread = None


    def loadParameters(self):
        # Topics
        self.sensors_output_topic = rospy.get_param("~topics/sensors_output_topic")
        self.pose_input_topic = rospy.get_param("~topics/pose_input_topic")
        self.status_output_topic = rospy.get_param("~topics/status_output_topic")

        rospy.loginfo("\n [{}]: Output topic = {}".format(rospy.get_name(), self.sensors_output_topic))
        rospy.loginfo("\n [{}]: Input topic = {}".format(rospy.get_name(), self.pose_input_topic))

        # Anchors
        a_list = rospy.get_param('~anchors')
        self.NumAnchors = len(a_list)
        rospy.loginfo("\n [{}] {} Anchors Loaded!".format(rospy.get_name(), self.NumAnchors))
        for i in range(self.NumAnchors):
            el = a_list[i]
            self.anchors[el['id']] = {
                        'pos': np.array([el['x'], el['y'], el['z']]),
                        'distorted': False,
                        'distortion': 0.0
                        }


    def initPubSub(self):
        # Anchor Message Publisher
        self.sensor_output_pub = rospy.Publisher(
                self.sensors_output_topic, AnchorMeas, queue_size=2)
        #Global status Publisher 
        self.status_output_pub = rospy.Publisher(
                self.status_output_topic, AnchorSimStatus, queue_size=2)

        # Subscribe to the vehicle pose
        rospy.Subscriber(self.pose_input_topic, PoseStamped, self.poseCallback)


    ###### CALLBACKS
    def poseCallback(self, pose_msg):
        # Update the position of the target
        self.target_pos[0] = pose_msg.pose.position.x;
        self.target_pos[1] = pose_msg.pose.position.y;
        self.target_pos[2] = pose_msg.pose.position.z;

        # Generate new measurements
        for anchor_id in range(self.NumAnchors):
            anchor_data = self.anchors[anchor_id]

            # Compute the distance
            anchor_pos = anchor_data['pos']
            anchor_meas = np.linalg.norm(self.target_pos - anchor_pos)

            # Add distortion if distorted
            if anchor_data['distorted']:
                anchor_meas = anchor_meas + anchor_data['distortion']

            # Compose the ROS message
            anch_msg = AnchorMeas()
            anch_msg.dist = anchor_meas
            anch_msg.id = anchor_id
            anch_msg.x_anchor = anchor_pos[0]
            anch_msg.y_anchor = anchor_pos[1]
            anch_msg.z_anchor = anchor_pos[2]

            # Add the measure to the group sensor data
            self.group_meas[anchor_id] = {'pos': anchor_pos, 'meas': anchor_meas, 'distortion': anchor_data['distortion'], 'distorted': anchor_data['distorted']}

            # Publish the message
            self.sensor_output_pub.publish(anch_msg)


    def status_thread(self):
        r = rospy.Rate(self.status_freq);
        while(self.active_thread and not rospy.is_shutdown()):
            msg = AnchorSimStatus()
            msg.header.stamp = rospy.Time.now()
            msg.anchors = []

            for (index, data) in self.group_meas.items():
                m = AnchorData()
                m.id = index
                m.pos = data['pos']
                m.meas = data['meas']
                m.distortion = data['distortion']
                m.isDistorted = data['distorted']
                msg.anchors.append(m)

            self.status_output_pub.publish(msg)
            r.sleep()


    def registerServices(self):
        # Advertise Services
        self.service_ctrl = rospy.Service('anchorSimCtrl',
                AnchorSimCtrl, self.handle_ctrl_req)

        self.service_status = rospy.Service('anchorSimStatus',
                AnchorSimTeleCtrl, self.handle_status_req)

   
    def handle_ctrl_req(self, req):
        anchor_id = req.id
        self.anchors[anchor_id]['enable'] = req.enable_anchor 
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
