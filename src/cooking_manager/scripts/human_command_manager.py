#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from controller.msg import SystemState

from human_command_manager.msg import SystemState, CurrentAction, valid_Command  

class HumanCommandManager:
    def __init__(self):
        # Initialize last command storage
        self._last_verbal_cmd = None
        self._current_state = None
        self._current_action = None
        
        # Subscribers
        rospy.Subscriber("/verbal_command", String, self._verbal_callback)
        rospy.Subscriber("/system_state", SystemState, self._state_callback)
        rospy.Subscriber("/current_action", CurrentAction, self._action_callback)
        
        # Publisher
        self._valid_cmd_pub = rospy.Publisher("/valid_Command", valid_Command, queue_size=10)
        
        # For testing purposes
        self._parsed_pub = rospy.Publisher("/parsed_command", String, queue_size=10)
    
    # --- Subscriber Callbacks ---
    def _verbal_callback(self, msg):
        """Store raw verbal command and process it."""
        self._last_verbal_cmd = msg.data
        parsed = self._parse_command(msg.data)
        if parsed:
            self._parsed_pub.publish(parsed)
            valid = self._validate_command(parsed)
            if valid:
                self._represent_command(parsed)

    def _state_callback(self, msg):
        """Update system state."""
        self._current_state = msg

    def _action_callback(self, msg):
        """Track current robot action."""
        self._current_action = msg

    # --- Core Methods ---
    def _parse_command(self, voice):
        """Convert raw speech-to-text output to structured command."""
        # Simple parsing logic - adapt as needed
        voice = voice.lower().strip()
        if "pass me the" in voice:
            obj = voice.replace("pass me the", "").strip()
            return {"action": "pass", "object": obj}
        elif "stop" in voice:
            return {"action": "stop", "object": None}
        return None

    def _represent_command(self, parsed_cmd):
        """Convert parsed command to ValidCommand message."""
        cmd = ValidCommand()
        cmd.action = parsed_cmd["action"]
        cmd.object = parsed_cmd["object"] if parsed_cmd["object"] else ""
        self._valid_cmd_pub.publish(cmd)

    def _validate_command(self, parsed_cmd):
        """Check if command is valid given current state."""
        # Basic validation - expand based on your needs
        if not parsed_cmd:
            return False
        if self._current_state and self._current_state.state == "EMERGENCY":
            return False
        return True