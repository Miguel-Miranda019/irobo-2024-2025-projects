#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped ##2nd one is new
from nav_msgs.msg import Path ##novo
from tf2_msgs.msg import TFMessage

class GroundTruthPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ground_truth_publisher')
        rospy.set_param('/use_sim_time', True)

        # Publisher for the PoseWithCovarianceStamped message
        self.publisher = rospy.Publisher('/gtInfo', PoseWithCovarianceStamped, queue_size=10)

        # Publisher for the Path message NOVO
        self.path_publisher = rospy.Publisher('/robot_path', Path, queue_size=10)
        
        # TF2 Buffer and Listener to retrieve transform data
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize the last publish time
        self.last_publish_time = rospy.Time(0)
        self.path_msg = Path() ##novo
        self.path_msg.header.frame_id = "mocap"  # Set the frame ID for the path NOVO
        self.path_msg.header.stamp = rospy.Time.now() ##novo

        # Subscribe to the TF messages
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        # Timer to publish the path at a fixed rate NOVO
        self.rate = rospy.Rate(10)  # 10 Hz for path publishing
        self.publish_path() ##novo

    def publish_pose(self, msg):
        """Publish the pose with covariance based on the provided transform message."""
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "mocap"
        pose.header.stamp = msg.header.stamp  # Use the incoming message's timestamp
        pose.pose.pose.position = msg.transform.translation  # Set position from transform
        pose.pose.pose.orientation = msg.transform.rotation  # Set orientation from transform
        
        # Set covariance matrix (identity or adjust based on your application)
        pose.pose.covariance = [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, 0.01
        ]
        
        self.publisher.publish(pose)  # Publish the pose
        rospy.loginfo("Published ground truth info.")
        
        ##TUDO NOVO
        # Create a PoseStamped message and add it to the path
        pose_stamped = PoseStamped()
        pose_stamped.header = pose.header
        pose_stamped.pose = pose.pose.pose  # Use the pose from PoseWithCovarianceStamped
        self.path_msg.poses.append(pose_stamped)  # Add the pose to the path

    ##NOVO
    def publish_path(self):
        """Publish the accumulated path at a fixed rate."""
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()  # Update the timestamp
            self.path_publisher.publish(self.path_msg)  # Publish the path
            self.rate.sleep()  # Sleep to maintain the loop rate
    
    def tf_callback(self, tf_msg):
        """Callback function to handle incoming TF messages."""
        try:
            # Iterate through all transforms in the message
            for transform in tf_msg.transforms:
                if transform.child_frame_id == 'mocap_laser_link':
                    # Check the timestamp of the transform
                    current_time = rospy.Time.now()
                    time_diff = current_time - self.last_publish_time

                    # If the time difference is more than 1 second
                    if time_diff.to_sec() >= 1:
                        self.publish_pose(transform)  # Call the publish_pose method
                        self.last_publish_time = current_time
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Extrapolation error in tf_callback: %s", str(e))
        except Exception as e:
            rospy.logwarn("Error in tf_callback: %s", str(e))

if __name__ == '__main__':
    try:
        gt_publisher = GroundTruthPublisher()
        rospy.spin()  # Keep the node running
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")