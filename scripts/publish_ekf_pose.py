#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros
import tf2_geometry_msgs

class PathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_ekf_publisher')
        rospy.set_param('/use_sim_time', True)

        # Publisher for the Path message
        self.path_publisher = rospy.Publisher('/robot_path_ekf', Path, queue_size=10)

        # Publisher for PoseWithCovarianceStamped messages
        self.pose_covariance_publisher = rospy.Publisher('/robot_pose_with_covariance_ekf', PoseWithCovarianceStamped, queue_size=10)

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize the Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "mocap"  # Set the frame ID for the path

        # Subscribe to the filtered odometry data
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)

        # Timer to publish the path at a fixed rate
        self.rate = rospy.Rate(10)  # 10 Hz for path publishing
        self.publish_path()

    def odometry_callback(self, odometry_msg):
        """Callback function to handle incoming Odometry messages."""

        try:
            # Lookup the transformation from 'map' to 'mocap'
            transform = self.tf_buffer.lookup_transform("mocap", "map", rospy.Time(0), rospy.Duration(1.0))

            # Create a PoseStamped message from the odometry message
            pose_stamped = PoseStamped()
            pose_stamped.header = odometry_msg.header
            pose_stamped.pose = odometry_msg.pose.pose  # Use the pose from Odometry

            # Transform the pose from the 'map' frame to the 'mocap' frame
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

            # Add the transformed pose to the path message
            self.path_msg.poses.append(transformed_pose)

            # Publish the pose with covariance in the 'mocap' frame
            self.publish_pose_with_covariance(odometry_msg, transformed_pose)

            rospy.loginfo("Processed and transformed pose.")

        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")

    def publish_pose_with_covariance(self, odometry_msg, transformed_pose):
        """Publish PoseWithCovarianceStamped in the transformed frame (mocap)."""

        # Create a PoseWithCovarianceStamped message
        pose_with_covariance_msg = PoseWithCovarianceStamped()
        pose_with_covariance_msg.header = transformed_pose.header
        pose_with_covariance_msg.pose.pose = transformed_pose.pose  # Use the transformed pose

        # Copy the covariance from the original odometry message
        pose_with_covariance_msg.pose.covariance = odometry_msg.pose.covariance

        # Publish the transformed PoseWithCovarianceStamped message
        self.pose_covariance_publisher.publish(pose_with_covariance_msg)

    def publish_path(self):
        """Publish the accumulated path at a fixed rate."""
        while not rospy.is_shutdown():
            # Update the header timestamp
            self.path_msg.header.stamp = rospy.Time.now()

            # Publish the path
            self.path_publisher.publish(self.path_msg)

            # Sleep to maintain the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        path_publisher = PathPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
