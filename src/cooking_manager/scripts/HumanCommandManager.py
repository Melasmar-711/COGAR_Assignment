
class HumanCommandManager:
    def __init__(self):
        # Subscribers
       """ rospy.Subscriber("/verbal_command", String, self._verbal_callback)
        rospy.Subscriber("/system_state", SystemState, self._state_callback)
        rospy.Subscriber("/current_action", CurrentAction, self._action_callback)"""
        
        # Publisher
       # self._valid_cmd_pub = rospy.Publisher("/valid_command", ValidCommand, queue_size=10)
        
     
    # --- Subscriber Callbacks ---
    def _verbal_callback(self, msg):
        """Store raw verbal command."""
        self._last_verbal_cmd = msg.data

    def _state_callback(self, msg):
        """Update system state (e.g., recipe step, urgency)."""

    def _action_callback(self, msg):
        """Track current robot action."""

    # --- Core Methods ---
    def _parse_command(self, voice):
        """Convert raw speech-to-text output to structured command."""
        # Example: "Pass me the salt" → {"action": "pass", "object": "salt"}
        return {text}

    def _represent_command(self, parsed_cmd):
        """  """

    def _validate_command(self, parsed_cmd):
        """Check if command aligns with system state and current action."""
       