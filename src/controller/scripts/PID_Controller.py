import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from assignments.srv import CheckJointState,CheckJointStateResponse
import random


class pid_controller:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)

        # dummy Parameters
        self.setpoint = 0.0
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05

        # State variables
        self.previous_error = 0.0
        self.integral = 0.0

        #rate
        self.rate = rospy.Rate(10)


        # Publisher
        self.pid_pub_pub = rospy.Publisher('/pid_output', Float64, queue_size=10)


        #create service client to get the trajectory to follow
        self.client = rospy.Service('/pid_get_trajectory', CheckJointState, self.input_callback)

        # Subscriber
        rospy.Subscriber('/error', Float64, self.input_callback)

    def input_callback(self, msg):
        """
        Callback function for the error input.
        """

        while(self.error>0.0006):

            self.pid_pub([random.uniform(0, 255)]*7)


        return CheckJointStateResponse(True, "Trajectory is reached")    


