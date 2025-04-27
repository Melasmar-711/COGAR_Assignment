#!/usr/bin/env python3
"""Human Command Manager Node.

This node manages human verbal commands by validating them against system state
and current action sequences. It coordinates between verbal commands, system state,
and action sequences to determine when commands can be safely executed.
"""

import rospy
from std_msgs.msg import String
from controller.msg import SystemState
from cooking_manager.srv import SendActionSeq, SendActionSeqResponse


class HumanCommandManager:
    """Manages validation and publishing of human commands based on system context.
    
    This class evaluates verbal commands against system state and current action
    sequences to determine if they can be safely executed, preventing conflicts
    with ongoing operations.
    """

    def __init__(self):
        """Initializes the HumanCommandManager with subscribers, services, and publishers."""
        self._last_verbal_cmd = None
        self._current_state = "IDLE"
        self.currnet_actions_sequence = []
        self.interrupted_already = False
        self._has_published = False

        # Subscribers
        rospy.Subscriber("/verbal_command", String, self.verbal_callback)
        rospy.Subscriber("/system_state", SystemState, self.state_callback)
        
        # a ros service of type sendActionSeq that recives the action sequence from the action_planner
        #wait for the action_sequence_to_command_monitor service to be available
        self._action_sequence_service = rospy.Service("action_sequence_to_command_monitor", SendActionSeq, self.actions_callback)

        # Publisher
        self._valid_pub = rospy.Publisher("/valid_command", String, queue_size=1)

    def verbal_callback(self, msg):
        """Callback for processing new verbal commands.
        
        Args:
            msg (String): The incoming verbal command message.
        """
        self._last_verbal_cmd = msg.data.strip()
        self._has_published = False
        rospy.loginfo(f"[RECEIVED] Verbal command: {self._last_verbal_cmd}")
        self.evaluate_and_publish()

    def state_callback(self, msg):
        """Callback for system state updates.
        
        Args:
            msg (SystemState): The current system state message.
        """
        self._current_state = msg.state.upper()
        # rospy.loginfo(f"[STATE] Updated: {self._current_state}")
        self.evaluate_and_publish()

    def actions_callback(self, msg):
        """Service callback for receiving action sequences.
        
        Args:
            msg (SendActionSeq): The incoming action sequence request.
            
        Returns:
            SendActionSeqResponse: Always returns True indicating successful receipt.
        """
        self.currnet_actions_sequence = [action.strip() for action in msg.action_sequence.split(',') if action.strip()]
       
        rospy.loginfo(f"[ACTIONS] Updated: {self.currnet_actions_sequence}")
        self.evaluate_and_publish()
        return SendActionSeqResponse(True)

    def evaluate_and_publish(self):
        """Evaluates current context and publishes valid commands when appropriate."""
        # Don't do anything if no new verbal command
        if not self._last_verbal_cmd or self._has_published:
            return

        if self._current_state == "IDLE":
            self.publish_valid_command(self._last_verbal_cmd)

        elif self._current_state == "EXECUTING":
            if self.is_non_urgent_ratio_low():
                self.publish_valid_command(self._last_verbal_cmd)
            else:
                rospy.loginfo("[REJECTED] Too many non-urgent actions. Command dropped.")

        elif self._current_state == "FAILURE":
            rospy.loginfo("[REJECTED] System in FAILURE. Command dropped.")

    def is_non_urgent_ratio_low(self):
        """Determines if the current action sequence has sufficient non-urgent actions.
        
        Returns:
            bool: True if ratio of non-urgent actions is above threshold (50%), False otherwise.
        """
        if not self.currnet_actions_sequence:
            return True  # If no actions, treat as low non-urgent

        non_urgent_count = sum(1 for action in self.currnet_actions_sequence if "not_urgent" in action)
        ratio = non_urgent_count / len(self.currnet_actions_sequence)
        rospy.loginfo(f"[EVAL] Non-urgent ratio: {ratio:.2f}")
        return ratio > 0.5  

    def publish_valid_command(self, command):
        """Publishes a validated command to the system.
        
        Args:
            command (str): The command to publish.
        """
        self._valid_pub.publish(String(data=command))
        self._has_published = True
        rospy.loginfo(f"[PUBLISHED] Valid Command: {command}")


if __name__ == "__main__":
    rospy.init_node("human_command_manager")
    rospy.loginfo("Human Command Manager Node Started")
    
    manager = HumanCommandManager()
    rospy.spin()
