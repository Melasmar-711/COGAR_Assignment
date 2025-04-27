#!/usr/bin/env python3
"""Action Planner Node.

This node translates recipe steps into actionable sequences and manages their execution.
It handles both automated recipe steps and human intervention commands, coordinating
between the recipe tracker, command monitor, and controller nodes.
"""

import rospy
from std_msgs.msg import String, Int32
from cooking_manager.msg import RecipeStep
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse, SendActionSeqRequest


class ActionPlanner:
    """Converts recipe steps to action sequences and manages command insertion.
    
    This class maintains the current action sequence, handles human intervention commands,
    and coordinates with other system components to execute cooking actions.
    """

    def __init__(self):
        """Initializes the ActionPlanner node with subscribers, services, and mappings."""
        rospy.init_node('action_planner', anonymous=True)
        rospy.loginfo("Action Planner: node started")

        self.current_step = ""
        self.prev_step = ""        

        self.valid_command = None
        self.prev_valid_command = None

        self.currnet_actions_sequence = None

        self.new_command_not_used = True
        self.seq_interrupted = False

        self.current_action_index = 0

        #set the rate of the loop
        self.rate = rospy.Rate(10)  # 10 Hz

        # Subscriber to the recipe step
        self.step_subscriber = rospy.Subscriber('recipe_step', RecipeStep, self.step_callback)

        # Subscriber to the valid command coming from human command manager
        self.valid_command_subscriber = rospy.Subscriber('valid_command', String, self.valid_command_callback)

        # Client to send the new action_sequence as a request to controller
        # wait for the action_sequence_to_controller service to be available
        rospy.wait_for_service('action_sequence_to_controller')
        self.action_sequence_client_to_controller = rospy.ServiceProxy('action_sequence_to_controller', SendActionSeq)
        
        #subscriber to the current step the controller is executing
        self.action_sequence_subscriber = rospy.Subscriber('action_index', Int32, self.change_action_index_callback)

        # Client to send the new action_sequence as a request to controller
        #wait for the action_sequence_to_command_monitor service to be available
        rospy.wait_for_service('action_sequence_to_command_monitor')
        self.action_sequence_client_to_command_monitor = rospy.ServiceProxy('action_sequence_to_command_monitor', SendActionSeq)

        self.steps_to_action_mapping = {
            "cut the onions": ["grab the onions urgent", "cut the onions not_urgent"],
            "cook onion": ["get onion not_urgent", "put in pan not_urgent", "turn on low heat not_urgent"],
            "boil the water": ["grab the pan not_urgent", "fill the pan with water urgent", "put the pan on the stove urgent"],
        }
        rospy.Timer(rospy.Duration(0.1), self.run)

    def step_callback(self, msg):
        """Callback for recipe step updates.
        
        Args:
            msg (RecipeStep): The current recipe step message.
        """
        self.current_step = msg.step

        if self.current_step != self.prev_step:
            self.currnet_actions_sequence = []
            self.current_action_index = 0

    def valid_command_callback(self, msg):
        """Callback for valid human commands.
        
        Args:
            msg (String): The validated human command message.
        """
        self.valid_command = msg.data

        if self.valid_command != self.prev_valid_command:
            self.new_command_not_used = True
            self.prev_valid_command = self.valid_command
            rospy.loginfo(f"Valid command received: {self.valid_command}")
        else:
            self.new_command_not_used = False 

    def change_action_index_callback(self, msg):
        """Callback for current action index updates.
        
        Args:
            msg (Int32): The current action index being executed.
        """
        self.current_action_index = msg.data

    def step_parsing(self, current_step):
        """Maps recipe steps to action sequences.
        
        Args:
            current_step (str): The current recipe step to parse.
        """
        rospy.loginfo(f"parsing: {current_step}")
        self.currnet_actions_sequence = self.steps_to_action_mapping.get(current_step, None)

    def re_arrange_sequence(self, action_sequence, new_valid_cmd):
        """Inserts human commands into the action sequence based on urgency.
        
        Args:
            action_sequence (list): Current sequence of actions.
            new_valid_cmd (str): Valid human command to insert.
        """
        # valid command inserting policy
        # if the number of non urgent actions remaining in the action sequence is more than 10% or equal to of the remaining actions, insert the new command at the end of the sequence
        # otherwise insert it at the end of the current action sequence
        non_urgent_actions = [action for action in action_sequence[self.current_action_index:] if "not_urgent" in action]

        if len(non_urgent_actions) / len(action_sequence[self.current_action_index:]) >= 0.1:
            if self.current_action_index <= len(action_sequence):
                self.currnet_actions_sequence.append(new_valid_cmd)

        self.new_command_not_used = False        

    def send_sequence(self):
        """Sends the current action sequence to controller and command monitor."""
        request = SendActionSeqRequest()

        # send the action sequence to the action planner topic
        action_sequence_str = ','.join(self.currnet_actions_sequence)
        request.action_sequence = action_sequence_str
        request.interrupted = self.seq_interrupted

        rospy.loginfo(f"Action sequence: {action_sequence_str}")
        try:
            self.action_sequence_client_to_command_monitor(request)
            self.action_sequence_client_to_controller(request)
            rospy.loginfo("Action sequence sent successfully")
            if self.seq_interrupted:
                self.currnet_actions_sequence.remove(self.valid_command)
            self.seq_interrupted = False
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def run(self, event):
        """Main execution loop for processing steps and commands.
        
        Args:
            event: Timer event (unused).
        """
        if (self.current_step != self.prev_step) or (self.new_command_not_used and self.valid_command != None):
            self.prev_step = self.current_step
            rospy.loginfo(f"about to parse: {self.current_step}")
            self.step_parsing(self.current_step)
            if self.currnet_actions_sequence is not None:
                # Check if the valid command is present
                if self.valid_command is not None and self.new_command_not_used: 
                    # Re-arrange the sequence based on the valid command
                    self.re_arrange_sequence(self.currnet_actions_sequence, self.valid_command)
                    rospy.loginfo(f"Re-arranged action sequence: {self.currnet_actions_sequence}")
                    self.seq_interrupted = True
                    self.new_command_not_used = False
                
                # Send the action sequence to the action planner topic
                self.send_sequence()


if __name__ == '__main__':
    action_planner = ActionPlanner()
    rospy.spin()
