#!/usr/bin/env python

import rospy

from assignments.srv import CheckJointState, CheckJointStateRequest
from std_msgs.msg import  Int32
from perception.msg import DetectedObjects  
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse
import random
from controller.msg import SystemState





class action_parser:
    def __init__(self):
        rospy.init_node('action_parser', anonymous=True)
        rospy.loginfo("action_parser: node started")

        self.objects={}
        self.received_sequences=[]
        self.action_index=0
        self.interrupted=False
        self.system_state=SystemState()
        self.system_state.state="IDLE"
        self.prev_action_sequence=[]


        # Create service client to get the trajectory to follow
        #wait for the service to be available
        rospy.loginfo("Waiting for /send_set_points service...")


        rospy.wait_for_service('/send_set_points')
        rospy.loginfo("Service /send_set_points is available")
        self.client_trajectory_send = rospy.ServiceProxy('/send_set_points', CheckJointState)


        #publisher for sending system state to human command monitor
        self.state_publisher=rospy.Publisher('/system_state', SystemState, queue_size=10)


        # Create a publisher to send the action index to the action planner so we know which action in the sequence we are executing
        self.action_index_publisher=rospy.Publisher('action_index', Int32, queue_size=10)


        #service to the recieve the sequence of actions from the action planner
        self.get_action_sequence_service= rospy.Service('/action_sequence_to_controller', SendActionSeq,self.sequence_callback)


        # Create a subscriber to get the objects of interest locations from perception node
        self.subscriber = rospy.Subscriber('/objects_of_interest', DetectedObjects, self.object_callback)



    def object_callback(self, msg):
        """
        Callback function for the detected objects.
        """     
        for i in range(len(msg.types)):
            object_type = msg.types[i]
            object_position = msg.poses.poses[i].position
            self.objects[object_type] = object_position
            #rospy.loginfo(f"Detected {object_type} at position {object_position}")

    def sequence_callback(self,msg):
        """
        This Callback function Used to recieve the action sequence from the action planner node.
        """

        # Convert the incoming string message to a list and append it to received_sequences
        new_seq=msg.action_sequence.split(",")
        self.interrupted=msg.interrupted

        self.received_sequences=new_seq

        if(not self.interrupted):
            self.action_index=0
        rospy.loginfo(f"Received action sequence: {self.received_sequences} with interrupted state: {self.interrupted}")
        return SendActionSeqResponse(True)
    


    


    def send_set_points(self):
        """
        This function is used to send the set points to the trajectory_manager.
        """
        
        #fill the CheckJointStateRequest with dummy velocities and positions and efforts for 7 joints
        #not based in any way on the action to be done itself just random values

        # Create a dummy CheckJointStateRequest
        joint_state_request = CheckJointStateRequest()
        joint_state_request.positions = [random.uniform(-1.0, 1.0) for _ in range(7)]
        joint_state_request.velocities = [random.uniform(-1.0, 1.0) for _ in range(7)]
        joint_state_request.efforts = [random.uniform(-1.0, 1.0) for _ in range(7)]

        success=self.client_trajectory_send(joint_state_request)



        #dummy logic to make failure states
        if not success and random.uniform(0, 50) < 0.5:
            self.system_state.state="FAILURE"
            self.state_publisher.publish(self.system_state)

        return success    



    def run(self):
        """
        Main loop for the action parser.
        """

        self.state_publisher.publish(self.system_state)



        while not rospy.is_shutdown():

            
            # Check if there are any received sequences
            if self.received_sequences!=self.prev_action_sequence : 

                self.prev_action_sequence=self.received_sequences

                # Process the latest received sequence
                for i in range(self.action_index,len(self.received_sequences)):
                    
                    #rospy.loginfo("legnth of received sequences: "+str(len(self.received_sequences)))
                    
                    self.action_index=i
                    self.action_index_publisher.publish(self.action_index)
                    self.system_state.state="EXECUTING"
                    self.state_publisher.publish(self.system_state)
                    sucess=self.send_set_points()


                    rospy.loginfo(f"interrupt : {self.interrupted} ,Processed action --{self.received_sequences[self.action_index]} --  in sequence  done :{sucess} ")
                    rospy.sleep(2)

                    if self.interrupted:
                        self.action_index+=1
                        break

            '''else:
                self.prev_action_sequence=self.received_sequences
                for i in range(self.action_index,len(self.received_sequences)):
                    self.action_index=i
                    self.action_index_publisher.publish(self.action_index)
                    self.system_state.state="EXECUTING"
                    self.state_publisher.publish(self.system_state)
                    sucess=self.send_set_points()
                   


                    rospy.loginfo(f"Processed action --{self.received_sequences[i]} --  in sequence  done :{sucess} ")
                    rospy.sleep(3)'''
                

            if self.interrupted==False:
                # If the sequence is not interrupted, reset the action index and state
                self.received_sequences = [] 
                self.system_state.state="IDLE"
                self.state_publisher.publish(self.system_state)
                #self.action_index=0
                self.action_index_publisher.publish(self.action_index)
                rospy.sleep(1)
            else:
                self.interrupted=False 
                rospy.sleep(1)
   



if __name__ == '__main__':
    try:
        action_parser_node = action_parser()
        action_parser_node.run()
    except rospy.ROSInterruptException:
        pass