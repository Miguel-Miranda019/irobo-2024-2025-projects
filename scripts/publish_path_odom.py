#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_publisher')
        rospy.set_param('/use_sim_time', True)
        
        # Publisher for the Path message
        self.path_publisher = rospy.Publisher('/robot_path_ekf', Path, queue_size=10)
        
        # Initialize the Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"  # Set the frame ID for the path
        
        # Subscribe to the filtered odometry data
        rospy.Subscriber('/odometry/filtered', Odometry, self.odometry_callback)
        
        # Timer to publish the path at a fixed rate
        self.rate = rospy.Rate(10)  # 10 Hz for path publishing
        self.publish_path()
        
    def odometry_callback(self, odometry_msg):
        """Callback function to handle incoming Odometry messages."""
        # Create a PoseStamped message from the odometry message
        pose_stamped = PoseStamped()
        pose_stamped.header = odometry_msg.header
        pose_stamped.pose = odometry_msg.pose.pose  # Use the pose from Odometry
        
        # Add the pose to the path
        self.path_msg.poses.append(pose_stamped)

        rospy.loginfo("Published /odometry/filtered info.")
        
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