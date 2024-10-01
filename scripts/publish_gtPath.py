#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class PathPublisher:
    def __init__(self):
        rospy.init_node('path_publisher')
        rospy.set_param('/use_sim_time', True)
        
        # Publisher for the Path message
        self.path_publisher = rospy.Publisher('/robot_path', Path, queue_size=10)
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initialize the Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "mocap"  # Set the frame ID for the path
        
        # Subscribe to the ground truth data
        rospy.Subscriber('/gtInfo', PoseWithCovarianceStamped, self.gt_callback)
        
        # Timer to publish the path at a fixed rate
        self.rate = rospy.Rate(10)  # 10 Hz for path publishing
        self.publish_path()
    
    def gt_callback(self, gt_msg):
        """Callback function to handle incoming ground truth messages."""
        
        pose_stamped = PoseStamped()
        pose_stamped.header = gt_msg.header
        pose_stamped.pose = gt_msg.pose.pose  # Use the pose from GT
        
        # Add the pose to the path
        self.path_msg.poses.append(pose_stamped)
        rospy.loginfo("Added ground truth pose to path.")
    
    def publish_path(self):
        """Publish the accumulated path at a fixed rate."""
        while not rospy.is_shutdown():
            self.path_msg.header.stamp = rospy.Time.now()  # Update the timestamp
            self.path_publisher.publish(self.path_msg)  # Publish the path
            self.rate.sleep()  # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        path_publisher = PathPublisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node Terminated")
