#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class DataSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('data_saver_node')
        rospy.set_param('/use_sim_time', True)
        
        # Subscribe to ground truth and EKF estimated pose topics
        rospy.Subscriber('/gtInfo', PoseWithCovarianceStamped, self.gt_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.ekf_callback)
        rospy.Subscriber('/robot_pose_with_covariance_ekf', PoseWithCovarianceStamped, self.ekf_transformed_callback)

        # Initialize variables to store data
        self.gt_pose_x = []
        self.gt_pose_y = []
        self.ekf_pose_x = []
        self.ekf_pose_y = []
        self.times = []
        self.uncertainties_x = []
        self.uncertainties_y = []
        
        self.initial_time = None

        # Filepath to save the data
        self.filepath = '/home/miguel/catkin_ws/src/turtlebot3_datasets/data/bag_data.npz'

        # Register the shutdown hook to save data when the node is terminated
        rospy.on_shutdown(self.save_data)

        rospy.loginfo("DataSaver node initialized. Waiting for data...")

    def gt_callback(self, gt_msg):
        """Callback function to handle ground truth pose."""
        # Log to check if callback is triggered
        rospy.loginfo("Received ground truth data")
        
        # Store the ground truth position
        self.gt_pose_x.append(gt_msg.pose.pose.position.x)
        self.gt_pose_y.append(gt_msg.pose.pose.position.y)


    def ekf_callback(self, ekf_msg):
        """Callback function to handle EKF estimated pose and covariance."""
        # Log to check if callback is triggered
        rospy.loginfo("Received EKF data")

        # Extract uncertainty from covariance matrix (x, y variances are on diagonal: indices 0, 7)
        uncertainty_x = np.sqrt(ekf_msg.pose.covariance[0])  # Standard deviation in x
        uncertainty_y = np.sqrt(ekf_msg.pose.covariance[7])  # Standard deviation in y

        self.uncertainties_x.append(uncertainty_x)
        self.uncertainties_y.append(uncertainty_y)
        
        current_time = ekf_msg.header.stamp.to_sec()

        # Set the initial time on the first message
        if self.initial_time is None:
            self.initial_time = current_time

        # Store the relative time
        relative_time = current_time - self.initial_time
        self.times.append(relative_time)

    
    def ekf_transformed_callback(self, ekf_msg):
        # Store EKF estimated position and covariance
        self.ekf_pose_x.append(ekf_msg.pose.pose.position.x)
        self.ekf_pose_y.append(ekf_msg.pose.pose.position.y)

    def save_data(self):
        """Save the collected data to a file."""
        rospy.loginfo("Attempting to save data...")

        if self.times:  # Check if there's data to save
            # Convert lists to numpy arrays and save them to file
            np.savez(self.filepath,
                     times=np.array(self.times),
                     gt_pose_x=np.array(self.gt_pose_x),
                     gt_pose_y=np.array(self.gt_pose_y),
                     ekf_pose_x=np.array(self.ekf_pose_x),
                     ekf_pose_y=np.array(self.ekf_pose_y),
                     uncertainties_x=np.array(self.uncertainties_x),
                     uncertainties_y=np.array(self.uncertainties_y))
            rospy.loginfo(f"Data saved to {self.filepath}")
        else:
            rospy.loginfo("No data to save.")

if __name__ == '__main__':
    try:
        data_saver = DataSaver()

        # Wait for ROS messages to process
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
