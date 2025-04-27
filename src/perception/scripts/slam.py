#!/usr/bin/env python
"""Dummy SLAM Node.

This node simulates a SLAM (Simultaneous Localization and Mapping) system by publishing
dummy map and odometry data in response to point cloud inputs. It serves as a placeholder
for actual SLAM algorithms during development and testing.
"""

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid, Odometry
import random

class DummySLAMNode:
    """Simulates SLAM functionality by generating dummy map and odometry data.
    
    This node publishes a static empty map and random robot positions to test
    navigation and mapping systems without requiring actual SLAM processing.
    """

    def __init__(self):
        """Initializes the dummy SLAM node with publishers and subscribers."""
        rospy.init_node('slam_node')

        # Publishers
        self.map_pub = rospy.Publisher('/slam/map', OccupancyGrid, queue_size=10)
        self.odom_pub = rospy.Publisher('/slam/odom', Odometry, queue_size=10)

        # Subscriber
        rospy.Subscriber('/cloud_points_fused', PointCloud2, self.cloud_callback)

        rospy.loginfo("Dummy SLAM Node Initialized")

    def cloud_callback(self, cloud_msg):
        """Callback for point cloud messages that triggers dummy SLAM outputs.
        
        Args:
            cloud_msg (PointCloud2): The incoming fused point cloud message.
        """
        # Every time we get new cloud data --> Publish dummy map + odom
        now = rospy.Time.now()

        # Dummy OccupancyGrid (simple empty map)
        map_msg = OccupancyGrid()
        map_msg.header.stamp = now
        map_msg.header.frame_id = "map"

        map_msg.info.resolution = 0.5  # 50 cm per cell
        map_msg.info.width = 20        # 20 cells
        map_msg.info.height = 20       # 20 cells
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        map_msg.data = [0 for _ in range(map_msg.info.width * map_msg.info.height)]  # all free space

        self.map_pub.publish(map_msg)
        # rospy.loginfo("Published dummy map")

        # Dummy Odometry (random position)
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = random.uniform(-2.0, 2.0)
        odom_msg.pose.pose.position.y = random.uniform(-2.0, 2.0)
        odom_msg.pose.pose.position.z = 5.0
        odom_msg.pose.pose.orientation.w = 1.0

        odom_msg.twist.twist.linear.x = 1.0
        odom_msg.twist.twist.linear.y = 10.0
        odom_msg.twist.twist.linear.z = 2.0
        odom_msg.twist.twist.angular.z = 5.0

        self.odom_pub.publish(odom_msg)
        # rospy.loginfo("Published dummy robot location")

    def run(self):
        """Main execution loop for the node."""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = DummySLAMNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
