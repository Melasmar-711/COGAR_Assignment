#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from cooking_manager.msg import RecipeStep



class ActionPlanner:
    '''
    This class is responsible for action planning based on the current step of the recipe.
    
    '''
    def __init__(self):

        rospy.init_node('action_planner', anonymous=True)
        self.current_step = ""
        self.previous_step = ""
        self.valid_command = None
        self.currnet_actions_sequence=[]
        self.new_command_used = False
        #set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscriber to the recipe step
        self.step_subscriber = rospy.Subscriber('recipe_step', RecipeStep, self.step_callback)
        self.valid_command_subscriber=rospy.Subscriber('valid_command', String, self.valid_command_callback)

        # Publisher to the action planner topic
        self.publisher = rospy.Publisher('action_sequence', String, queue_size=50)

        self.steps_to_action_mapping = {"cut the onions": ["grab the onions urgent", "cut the onions not_urgent"],
                                        "cook onion": ["get onion not_urgent", "put in pan not_urgent", "turn on low heat not_urgent"],
                                        "boil the water":["grab the pan not_urgent", "fill the pan with water urgent", "put the pan on the stove urgent"],
                                        }




    def step_callback(self, msg):
        '''retrieve the current step from the recipe tracker'''
        self.current_step = msg.step
        rospy.loginfo(f"Received step: {self.current_step}")

    def valid_command_callback(self, msg):
        '''retrieve the valid command from the human command manager'''
        self.valid_command = msg.data
        self.new_command_used = False


   
    def step_parsing(self, current_step):
        '''parse the current step and retrieve the corresponding action sequence'''
        rospy.loginfo(f"parsing: {current_step}")
        self.currnet_actions_sequence=self.steps_to_action_mapping.get(current_step,None)


    # One function to decide and send/publish the current action
    def re_arrange_sequence(self,action_sequence,new_valid_cmd):

        # valid command inserting policy
        non_urgent_actions = [action for action in action_sequence if "not_urgent" in action]
        if len(non_urgent_actions) / len(action_sequence) > 0.6:
            for i, action in enumerate(action_sequence):
                if "not_urgent" in action:
                    self.currnet_actions_sequence.insert(i, new_valid_cmd)
                    break
    
    def send_sequence(self):
        # Publish the action sequence to the action planner topic
        rospy.loginfo(f"Sending action sequence: {self.currnet_actions_sequence}")
        action_sequence_str = ','.join(self.currnet_actions_sequence)
        rospy.loginfo(f"Action sequence: {action_sequence_str}")
        self.publisher.publish(action_sequence_str)


    def run(self):

        while not rospy.is_shutdown():

            # Check if the current step is different from the previous step
            
            

            if self.current_step != self.previous_step:
                self.previous_step = self.current_step
                rospy.loginfo(f"about to parse: {self.current_step}")
                self.step_parsing(self.current_step)
                if self.currnet_actions_sequence is not None:
                    # Check if the valid command is present
                    if self.valid_command is not None:
                        # Re-arrange the sequence based on the valid command
                        self.re_arrange_sequence(self.currnet_actions_sequence, self.valid_command)
                        self.new_command_used = True

                    # Send the action sequence to the action planner topic
                    self.send_sequence()
            else:
                rospy.loginfo("No new step received.")

            # Sleep to maintain the loop rate
            self.rate.sleep()    

            

if __name__ == '__main__':
    action_planner = ActionPlanner()
    action_planner.run()
