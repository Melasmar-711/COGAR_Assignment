class ControllerSubsystem:
    def __init__(self):
        """Initialize subscribers and publishers"""
        # Subscribers
        """ rospy.Subscriber("/object_location", ObjectLocation, self._object_location_callback)
        rospy.Subscriber("/map", Map, self._map_callback)
        rospy.Subscriber("/current_force", ForceData, self._force_callback)
        
        # Publishers
        self._motor_cmd_pub = rospy.Publisher("/motor_commands", MotorCommand, queue_size=10)
        self._action_state_pub = rospy.Publisher("/action_state", ActionState, queue_size=10)
        """
        pass
    # --- Callbacks ---
    def _object_location_callback(self, msg: ObjectLocation) -> None:
        """Store latest object position
        Input: ObjectLocation message (x,y,z coordinates)
        Output: None (updates internal state)
        """
        pass
    def _map_callback(self, msg: Map) -> None:
        """Update environment map
        Input: Map message (occupancy grid)
        Output: None (updates internal state)
        """
        pass

    def _force_callback(self, msg: ForceData) -> None:
        """Monitor force sensor readings
        Input: ForceData message (force vector)
        Output: None (updates internal state)
        """
        pass

    # --- Core Components ---
    def _action_parsing(self) -> Tuple[ActionState, List[Point]]:
        """Convert object location to executable actions
        Input: None (uses internal object_location state)
        Output: (ActionState, [setpoints]) - Current action type and target positions
        """
        return None

    def _trajectory_manager(self, setpoints: List[Point], map: Map) -> Trajectory:
        """Generate collision-free path
        Input: [setpoints], environment Map
        Output: Trajectory (path waypoints)
        """
        return None

    def _pid_controller(self, trajectory: Trajectory, 
                       current_pose: Pose, 
                       force_data: ForceData) -> MotorCommand:
        """Compute motor commands
        Input: (Trajectory, current Pose, ForceData)
        Output: MotorCommand (velocity, torque for each joint)
        """
        return None

    # --- Main Loop ---
    def run(self) -> None:
        """Execute control pipeline at fixed rate
        Input: None
        Output: None (publishes to /motor_commands and /action_state)
        """
        return None
