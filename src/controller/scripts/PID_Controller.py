#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from assignments.srv import CheckJointState,CheckJointStateResponse
import random
from control_msgs.msg import JointTrajectoryControllerState

class pid_controller:
    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        rospy.loginfo("pid_controller: node started")

        # dummy Parameters
        self.setpoint = 0.0
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.05
        self.error = 10.0
        self.previous_error = 10.0
        self.integral = 0.0

        #rate
        self.rate = rospy.Rate(10)


        # publisher to output dummy values as long as the error is above a dummy threshold
        self.pid_pub= rospy.Publisher('/pid_output', Float64, queue_size=10)


        #a service to retrive dummy trajectories from the trajectory_manager node
        self.client = rospy.Service('/pid_get_trajectory', CheckJointState, self.input_callback)



    def input_callback(self, msg):
        """
        Callback function for the error input.
        """

        
        rospy.loginfo("pid_controller: waiting for arm states ")
        bag_msg = rospy.wait_for_message('/arm_right_controller/state', JointTrajectoryControllerState)

        rospy.loginfo("pid_controller: arm states received")
        self.error=bag_msg.error.positions    
        

        timeout = rospy.Time.now() + rospy.Duration(2)  # 10 seconds timeout
        while rospy.Time.now() < timeout:


            rospy.sleep(2)  # Sleep for a short duration to avoid busy waiting

            # if 1st joint reached an error of less than 0.0007s consider pose reached (dummy)
            if abs(self.error[0]) < 0.007:
                rospy.loginfo("pid_controller: pose reached and error is less than threshold")
                return CheckJointStateResponse(True)    
    
            self.pid_pub.publish([random.uniform(0, 255)] * 7)
            rospy.loginfo("pid_controller: publishing dummy pid output")


            self.rate.sleep()    
        
        rospy.loginfo("pid_controller: pose not reached")
        return CheckJointStateResponse(False)

       



if __name__ == '__main__':
    try:
        pid_controller = pid_controller()
        while not rospy.is_shutdown():
            # Initialize the PID controller
            pass

    except rospy.ROSInterruptException:
        pass