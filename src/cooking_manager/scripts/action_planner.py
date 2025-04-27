#!/usr/bin/env python3

import rospy
from std_msgs.msg import String ,Int32
from cooking_manager.msg import RecipeStep
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse, SendActionSeqRequest



class ActionPlanner:
    '''
    This class is responsible for action planning based on the current step of the recipe.
    
    '''
    def __init__(self):

        rospy.init_node('action_planner', anonymous=True)
        rospy.loginfo("Action Planner: node started")

        self.current_step = ""
        self.prev_step=""        

        self.valid_command = None
        self.prev_valid_command=None

        self.currnet_actions_sequence=None


        self.new_command_not_used = True
        self.seq_interrupted=False


        self.current_action_index=0
        


        #set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz


        # Subscriber to the recipe step
        self.step_subscriber = rospy.Subscriber('recipe_step', RecipeStep, self.step_callback)

        # Subscriber to the valid command coming from human command manager
        self.valid_command_subscriber=rospy.Subscriber('valid_command', String, self.valid_command_callback)


        # Client to send the new  action_sequence as  a request to controller
        # wait for the action_sequence_to_controller service to be available

        rospy.wait_for_service('action_sequence_to_controller')
        self.action_sequence_client_to_controller = rospy.ServiceProxy('action_sequence_to_controller', SendActionSeq)


        
        #subscriber to the current step the controller is executing
        self.action_sequence_subscriber = rospy.Subscriber('action_index',Int32 , self.change_action_index_callback)


        # Client to send the new  action_sequence as  a request to controller
        #wait for the action_sequence_to_command_monitor service to be available
        rospy.wait_for_service('action_sequence_to_command_monitor')
        self.action_sequence_client_to_command_monitor = rospy.ServiceProxy('action_sequence_to_command_monitor', SendActionSeq)



        self.steps_to_action_mapping = {"cut the onions": ["grab the onions urgent", "cut the onions not_urgent"],
                                        "cook onion": ["get onion not_urgent", "put in pan not_urgent", "turn on low heat not_urgent"],
                                        "boil the water":["grab the pan not_urgent", "fill the pan with water urgent", "put the pan on the stove urgent"],
                                        }




    def step_callback(self, msg):
        '''retrieve the current step from the recipe tracker'''
        self.current_step = msg.step

        if(self.current_step != self.prev_step):
            self.currnet_actions_sequence=[]
            self.current_action_index=0

        self.valid_command=None



    def valid_command_callback(self, msg):
        '''retrieve the valid command from the human command manager'''

        self.valid_command = msg.data

        if self.valid_command != self.prev_valid_command:
            
            self.new_command_not_used = True
            self.seq_interrupted=True
            self.prev_valid_command = self.valid_command
        
            rospy.loginfo(f"Valid command received: {self.valid_command}")


    def change_action_index_callback(self, msg):
        self.current_action_index = msg.data

   
    def step_parsing(self, current_step):
        '''parse the current step and retrieve the corresponding action sequence'''
        rospy.loginfo(f"parsing: {current_step}")
        self.currnet_actions_sequence=self.steps_to_action_mapping.get(current_step,None)


    # One function to decide and send/publish the current action
    def re_arrange_sequence(self,action_sequence,new_valid_cmd):

        # valid command inserting policy
        # if the number of non urgent actions remaining in the action sequence is more than 50% or equal to of the remaining actions, insert the new command at the end of the sequence
        # otherwise insert it at the end of the current action sequence
        non_urgent_actions = [action for action in action_sequence[self.current_action_index:] if "not_urgent" in action]

        if len(non_urgent_actions) / len(action_sequence[self.current_action_index:]) >= 0.1:
            if "not_urgent"in action_sequence[self.current_action_index +1]:
                # Insert the new command at the end of the sequence
                action_sequence.insert(self.current_action_index + 1, new_valid_cmd)
            else:
                action_sequence.append(new_valid_cmd)



    
    def send_sequence(self):
        
        request=SendActionSeqRequest()
        
        

        # send the action sequence to the action planner topic

        action_sequence_str = ','.join(self.currnet_actions_sequence)

        request.action_sequence = action_sequence_str
        request.interrupted=self.seq_interrupted
        rospy.loginfo(f"Action sequence: {action_sequence_str}")
        try:
            self.action_sequence_client_to_command_monitor(request)
            self.action_sequence_client_to_controller(request)
            rospy.loginfo("Action sequence sent successfully")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")




    def run(self):

        while not rospy.is_shutdown():

            #rospy.wait_for_service('action_sequence_to_command_monitor')

            if (self.current_step != self.prev_step ) or (self.new_command_not_used and self.valid_command is not None):
                self.prev_step = self.current_step
                rospy.loginfo(f"about to parse: {self.current_step}")
                self.step_parsing(self.current_step)
                if self.currnet_actions_sequence is not None:
                    # Check if the valid command is present
                    if self.valid_command is not None :
                        # Re-arrange the sequence based on the valid command
                        self.re_arrange_sequence(self.currnet_actions_sequence, self.valid_command)

                        self.seq_interrupted=True
                        self.new_command_not_used = False
                        
                        
                        

                    # Send the action sequence to the action planner topic
                    self.send_sequence()
            else:
                rospy.loginfo("No new step to process.")

            # Sleep to maintain the loop rate
            self.rate.sleep()    

            

if __name__ == '__main__':
    action_planner = ActionPlanner()
    action_planner.run()
