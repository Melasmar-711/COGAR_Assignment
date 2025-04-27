#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, Range, Image, PointCloud2

class SensorFusionNode:
    def __init__(self):
        rospy.init_node('sensor_fusion_node')

        # Initialize last received times
        self.lidar_time = None
        self.sonar_time = None
        self.depth_time = None
        self.cloud_time = None

        # Storage for latest sonar message
        self.latest_cloud_point_msg = None

        # Subscribers
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/sonar_base', Range, self.sonar_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/cloud_points', PointCloud2, self.cloud_callback)

        # Publisher
        self.pub = rospy.Publisher('/cloud_points_fused', PointCloud2, queue_size=10)

        # Timer to check and publish if all data is updated
        rospy.Timer(rospy.Duration(0.05), self.check_and_publish)

    def lidar_callback(self, msg):
        self.lidar_time = rospy.Time.now()

    def sonar_callback(self, msg):
        self.sonar_time = rospy.Time.now()

    def depth_callback(self, msg):
        self.depth_time = rospy.Time.now()

    def cloud_callback(self, msg):
        self.latest_cloud_point_msg = msg
        self.cloud_time = rospy.Time.now()

    def check_and_publish(self, event):
        if None in (self.lidar_time, self.sonar_time, self.depth_time, self.cloud_time):
            # Not all sensors received yet
            return

        # Define a max allowed delay (e.g., 0.2 seconds)
        max_delay = rospy.Duration(0.2)

        now = rospy.Time.now()

        if (now - self.lidar_time < max_delay and
            now - self.sonar_time < max_delay and
            now - self.depth_time < max_delay and
            now - self.cloud_time < max_delay):

            if self.latest_cloud_point_msg:
                self.pub.publish(self.latest_cloud_point_msg)
                # rospy.loginfo("Published fused point cloud")


if __name__ == '__main__':
    try:
        node = SensorFusionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
