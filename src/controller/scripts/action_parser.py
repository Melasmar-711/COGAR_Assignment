#!/usr/bin/env python

import rospy
from assignments.srv import CheckJointState, CheckJointStateRequest
from std_msgs.msg import Int32
from perception.msg import DetectedObjects  
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse
import random
from controller.msg import SystemState

class action_parser:
    """Parses and executes action sequences for the robot controller.
    
    This node receives action sequences from the planner, manages their execution,
    and coordinates with other system components including trajectory management
    and state monitoring.
    """

    def __init__(self):
        """Initializes the action parser node with services, publishers, and subscribers."""
        rospy.init_node('action_parser', anonymous=True)
        rospy.loginfo("action_parser: node started")

        self.objects = {}
        self.received_sequences = []
        self.action_index = 0
        self.interrupted = False
        self.system_state = SystemState()
        self.system_state.state = "IDLE"
        self.prev_action_sequence = []

        # Create service client to get the trajectory to follow
        rospy.loginfo("Waiting for /send_set_points service...")
        rospy.wait_for_service('/send_set_points')
        rospy.loginfo("Service /send_set_points is available")
        self.client_trajectory_send = rospy.ServiceProxy('/send_set_points', CheckJointState)

        # Publisher for sending system state to human command monitor
        self.state_publisher = rospy.Publisher('/system_state', SystemState, queue_size=10)

        # Publisher for action index
        self.action_index_publisher = rospy.Publisher('action_index', Int32, queue_size=10)

        # Service to receive action sequences
        self.get_action_sequence_service = rospy.Service('/action_sequence_to_controller', SendActionSeq, self.sequence_callback)

        # Subscriber for detected objects
        self.subscriber = rospy.Subscriber('/objects_of_interest', DetectedObjects, self.object_callback)
        
        # Timers
        rospy.Timer(rospy.Duration(.1), self.publishing)
        rospy.Timer(rospy.Duration(1.0), self.run)

    def object_callback(self, msg):
        """Callback for detected objects from perception node.
        
        Args:
            msg (DetectedObjects): Message containing detected objects and their positions.
        """
        for i in range(len(msg.types)):
            object_type = msg.types[i]
            object_position = msg.poses.poses[i].position
            self.objects[object_type] = object_position
            #rospy.loginfo(f"Detected {object_type} at position {object_position}")

    def sequence_callback(self, msg):
        """Service callback for receiving action sequences from planner.
        
        Args:
            msg (SendActionSeq): Service request containing the action sequence.
            
        Returns:
            SendActionSeqResponse: Always returns True indicating successful receipt.
        """
        # Convert the incoming string message to a list and append it to received_sequences
        new_seq = msg.action_sequence.split(",")
        self.interrupted = msg.interrupted

        self.received_sequences = new_seq

        if not self.interrupted:
            self.action_index = 0
        rospy.loginfo(f"Received action sequence: {self.received_sequences} with interrupted state: {self.interrupted}")
        return SendActionSeqResponse(True)

    def send_set_points(self):
        """Sends joint set points to trajectory manager.
        
        Returns:
            bool: True if set points were sent successfully, False otherwise.
        """
        # Create a dummy CheckJointStateRequest
        joint_state_request = CheckJointStateRequest()
        joint_state_request.positions = [random.uniform(-1.0, 1.0) for _ in range(7)]
        joint_state_request.velocities = [random.uniform(-1.0, 1.0) for _ in range(7)]
        joint_state_request.efforts = [random.uniform(-1.0, 1.0) for _ in range(7)]

        success = self.client_trajectory_send(joint_state_request)

        # Dummy logic to make failure states
        if not success and random.uniform(0, 50) < 0.5:
            self.system_state.state = "FAILURE"
            self.state_publisher.publish(self.system_state)

        return success

    def publishing(self, event):
        """Periodically publishes system state and action index.
        
        Args:
            event: Timer event (unused).
        """
        rospy.loginfo(f"current action sequence: {self.received_sequences}")
        rospy.loginfo(f"system state: {self.system_state}")
        self.state_publisher.publish(self.system_state)
        self.action_index_publisher.publish(self.action_index)
    
    def run(self, event):
        """Main execution loop for processing action sequences.
        
        Args:
            event: Timer event (unused).
        """
        if self.received_sequences is None:
            self.system_state.state = "IDLE"
            return
        
        if self.action_index + 1 >= len(self.received_sequences):
            self.received_sequences = None
        else:
            self.system_state.state = "EXECUTING"
            sucess = self.send_set_points()
            self.action_index += 1

if __name__ == '__main__':
    try:
        action_parser_node = action_parser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
