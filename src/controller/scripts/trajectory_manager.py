#!/usr/bin/env python
"""Trajectory Manager Node.

This node manages robot trajectories by coordinating between action planning,
pose reachability checking, and PID control. It receives target positions,
verifies reachability, and sends trajectories to the PID controller.
"""

import rospy
from assignments.srv import CheckJointState, CheckJointStateRequest, CheckJointStateResponse
from nav_msgs.msg import OccupancyGrid, Odometry

class trajectory_manager:
    """Manages robot trajectory execution through coordination with other nodes.
    
    This node serves as an intermediary between action planning and low-level control,
    handling trajectory validation and execution monitoring.
    """

    def __init__(self):
        """Initializes the trajectory manager with services and subscribers."""
        rospy.init_node('trajectory_manager_', anonymous=True)
        rospy.loginfo("trajectory_manager: node started")

        self.odom = Odometry()
        self.map_msg = OccupancyGrid()
        self.check_joint_state = CheckJointStateRequest()    
        self.rate = rospy.Rate(10)

        # Create service client to get the trajectory to follow
        # this service will be used to receive the set points from the action_parser as a service request
        self.client_recieve_set_points = rospy.Service('/send_set_points', CheckJointState, self.input_callback)

        # Create a service client to send the trajectory to the PID controller
        rospy.loginfo("Waiting for /pid_get_trajectory service...")
        rospy.wait_for_service('/pid_get_trajectory')
        # this service client will be used to send the dummy trajectory to the pid node as a service request
        self.client_trajectory_send = rospy.ServiceProxy('/pid_get_trajectory', CheckJointState)

        # subscribers to the map and odometry topics coming from the perception node
        self.map_sub = rospy.Subscriber('/slam/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/slam/odom', Odometry, self.odom_callback)

        # make a service client to check if the position is reachable
        rospy.loginfo("Waiting for /check_reachable_pose service...")
        rospy.wait_for_service('/check_reachable_pose')
        rospy.loginfo("Service /check_reachable_pose is available")

        # this is the service provided in the assignment , it has dummy checks for whether positions are reachable or not
        # we use this as a first check in the trajectory manager . if not reachable we dont send the trajectory to the pid controller
        # if reachable we send the trajectory to the pid controller and wait for the pid to try
        self.client_check_reachable_pose = rospy.ServiceProxy('/check_reachable_pose', CheckJointState)

    def input_callback(self, msg):
        """Service callback for receiving set points from action parser.
        
        Args:
            msg (CheckJointState): The incoming trajectory request.
            
        Returns:
            CheckJointStateResponse: True if trajectory was successfully executed, False otherwise.
        """
        rospy.loginfo("trajectory_manager: recived new set points") 
        self.check_joint_state = msg

        # check using the dummy service whether the recieved set points are reachable or not
        sucess = self.client_check_reachable_pose(self.check_joint_state)

        if sucess.success:  # if reachable send to the pid controller
            reached = self.send_trajectory() # wait for the pid controller to reach the set points or fail
            if reached.success: # if the pid controller reached the set points successfully
                rospy.loginfo("trajectory_manager: pid says pose reached successfully")
                return CheckJointStateResponse(True)
            else:
                rospy.loginfo("trajectory_manager: pid says pose couldn't be reached successfully")
                return CheckJointStateResponse(False)
        else:
            rospy.loginfo("trajectory_manager: pose not reachable")
            return CheckJointStateResponse(False)

    def map_callback(self, data):
        """Callback for receiving map updates from SLAM.
        
        Args:
            data (OccupancyGrid): The incoming map message.
        """
        # Dummy OccupancyGrid (simple empty map)
        self.map_msg.header.stamp = data.header.stamp
        self.map_msg.header.frame_id = data.header.frame_id
        self.map_msg.info.resolution = data.info.resolution
        self.map_msg.info.width = data.info.width
        self.map_msg.info.height = data.info.height
        self.map_msg.info.origin.position.x = data.info.origin.position.x
        self.map_msg.info.origin.position.y = data.info.origin.position.y
        self.map_msg.info.origin.position.z = data.info.origin.position.z
        self.map_msg.info.origin.orientation.w = data.info.origin.orientation.w
        self.map_msg.data = data.data

    def odom_callback(self, data):
        """Callback for receiving odometry updates.
        
        Args:
            data (Odometry): The incoming odometry message.
        """
        # Dummy Odometry (random position)
        self.odom.header.stamp = data.header.stamp
        self.odom.header.frame_id = data.header.frame_id
        self.odom.child_frame_id = data.child_frame_id
        self.odom.pose.pose.position.x = data.pose.pose.position.x
        self.odom.pose.pose.position.y = data.pose.pose.position.y
        self.odom.pose.pose.position.z = data.pose.pose.position.z
        self.odom.pose.pose.orientation.w = data.pose.pose.orientation.w
        self.odom.twist.twist.linear.x = data.twist.twist.linear.x
        self.odom.twist.twist.linear.y = data.twist.twist.linear.y
        self.odom.twist.twist.linear.z = data.twist.twist.linear.z
        self.odom.twist.twist.angular.z = data.twist.twist.angular.z

    def send_trajectory(self):
        """Sends trajectory to PID controller for execution.
        
        Returns:
            CheckJointStateResponse: Response from PID controller indicating success.
        """
        successfully_reached = self.client_trajectory_send(self.check_joint_state)

        if successfully_reached:
            rospy.loginfo("trajectory_manager: trajectory sent successfully")
        else:
            rospy.loginfo("trajectory_manager: trajectory not sent successfully")

        return successfully_reached    

    def run(self):
        """Main execution loop for the trajectory manager."""
        while not rospy.is_shutdown():
            rospy.sleep(0.5)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        trajectory_manager_node = trajectory_manager()
        trajectory_manager_node.run()
    except rospy.ROSInterruptException:
        pass
