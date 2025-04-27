#!/usr/bin/env python
"""Computer Vision Node.

This node simulates computer vision processing by publishing dummy point cloud data
and detected objects. It serves as a placeholder for actual computer vision algorithms,
providing test data for other system components.
"""

import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray
from perception.msg import DetectedObjects  
import std_msgs.msg

class ComputerVisionNode:
    """Simulates computer vision processing for the cooking robot system.
    
    Publishes dummy point cloud data and detected objects to test system integration.
    """

    def __init__(self):
        """Initializes the computer vision node with publishers and subscribers."""
        rospy.init_node('computer_vision_node')

        # Publishers
        self.cloud_pub = rospy.Publisher('/cloud_points', PointCloud2, queue_size=10)
        self.objects_pub = rospy.Publisher('/objects_of_interest', DetectedObjects, queue_size=10)

        # Subscriber
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.rgbd_callback)

        rospy.loginfo("Computer Vision Node Initialized")

    def rgbd_callback(self, msg):
        """Callback for RGBD image processing.
        
        Args:
            msg (Image): The incoming RGBD image message.
        """
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera_link"

        # Publish dummy cloud points
        cloud_points = [
            (1.0, 0.0, 2.0),
            (1.5, 0.5, 2.5),
            (2.0, -0.5, 3.0),
            (2.5, 0.0, 3.5)
        ]
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_points)
        self.cloud_pub.publish(cloud_msg)
        # rospy.loginfo("Published dummy cloud points")

        # Publish dummy objects of interest
        detected_objects = DetectedObjects()

        poses = PoseArray()
        poses.header = header

        dummy_objects = [
            {"position": (1.2, 0.2, 2.1), "type": "pan"},
            {"position": (2.3, 0.1, 3.4), "type": "onion"},
        ]

        for obj in dummy_objects:
            pose = Pose()
            pose.position.x = obj["position"][0]
            pose.position.y = obj["position"][1]
            pose.position.z = obj["position"][2]
            pose.orientation.w = 1.0  # No rotation
            poses.poses.append(pose)
            detected_objects.types.append(obj["type"])

        detected_objects.poses = poses

        self.objects_pub.publish(detected_objects)
        # rospy.loginfo("Published dummy DetectedObjects")

    def run(self):
        """Main execution loop for the node."""
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ComputerVisionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
