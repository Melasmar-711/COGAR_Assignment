#!/usr/bin/env python



import rospy
from assignments.srv import CheckJointState,CheckJointStateRequest,CheckJointStateResponse
from nav_msgs.msg import OccupancyGrid, Odometry


class trajectory_manager:


    def __init__(self):
        rospy.init_node('trajectory_manager_', anonymous=True)
        rospy.loginfo("trajectory_manager: node started")

        self.odom = Odometry()
        self.map_msg = OccupancyGrid()
        self.check_joint_state = CheckJointStateRequest()    
        # Rate
        self.rate = rospy.Rate(10)


        # Create service client to get the trajectory to follow
        self.client_recieve_set_points = rospy.Service('/send_set_points', CheckJointState,self.input_callback)

        # Create a service client to send the trajectory to the PID controller
        # wait for the service to be available
        rospy.loginfo("Waiting for /pid_get_trajectory service...")
        rospy.wait_for_service('/pid_get_trajectory')
        self.client_trajectory_send = rospy.ServiceProxy('/pid_get_trajectory', CheckJointState)


        # Publishers
        self.map_sub= rospy.Subscriber('/slam/map', OccupancyGrid,self.map_callback)
        self.odom_sub = rospy.Subscriber('/slam/odom', Odometry, self.odom_callback)

        # make a service client to check if the position is reachable
        #wait for the service to be available
        rospy.loginfo("Waiting for /check_reachable_pose service...")
        rospy.wait_for_service('/check_reachable_pose')
        rospy.loginfo("Service /check_reachable_pose is available")


        self.client_check_reachable_pose = rospy.ServiceProxy('/check_reachable_pose', CheckJointState)


    def input_callback(self, msg):

        rospy.loginfo("trajectory_manager: recived new set points") 
        self.check_joint_state=msg
        sucess=self.client_check_reachable_pose(self.check_joint_state)
        if sucess.success:

            reached=self.send_trajectory()
            if  reached.success:
                rospy.loginfo("trajectory_manager: pid says pose reached successfully")
                return CheckJointStateResponse(True)
            else:
                rospy.loginfo("trajectory_manager: pid says pose couldn't be reached successfully")

                return CheckJointStateResponse(False)
            
        else:
            rospy.loginfo("trajectory_manager: pose not reachable")
            return CheckJointStateResponse(False)    







    def map_callback(self,data):
        # Dummy OccupancyGrid (simple empty map)
        self.map_msg.header.stamp=data.header.stamp
        self.map_msg.header.frame_id=data.header.frame_id
        self.map_msg.info.resolution = data.info.resolution
        self.map_msg.info.width = data.info.width
        self.map_msg.info.height = data.info.height
        self.map_msg.info.origin.position.x = data.info.origin.position.x
        self.map_msg.info.origin.position.y = data.info.origin.position.y
        self.map_msg.info.origin.position.z = data.info.origin.position.z
        self.map_msg.info.origin.orientation.w = data.info.origin.orientation.w
        self.map_msg.data = data.data

        #rospy.loginfo("trajectory_manager: recieved map data")
    

    def odom_callback(self,data):
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
        #rospy.loginfo("trajectory_manager: recieved odometry data")

    def send_trajectory(self):
        """
        Send the trajectory to the PID controller.
        """

        successfully_reached=self.client_trajectory_send(self.check_joint_state)
        if successfully_reached:
            rospy.loginfo("trajectory_manager: trajectory sent successfully")
        else:
            rospy.loginfo("trajectory_manager: trajectory not sent successfully")

        return successfully_reached    

    def run(self):
        """
        Main loop for the trajectory manager.
        """
        while not rospy.is_shutdown():
            
            rospy.sleep(0.5)
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        trajectory_manager_node = trajectory_manager()
        trajectory_manager_node.run()
    except rospy.ROSInterruptException:
        pass            