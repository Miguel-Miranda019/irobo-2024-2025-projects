#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt

class ErrorCalculator:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('evaluation_node')
        rospy.set_param('/use_sim_time', True)
        
        # Subscribe to ground truth and EKF estimated pose topics
        rospy.Subscriber('/gtInfo', PoseWithCovarianceStamped, self.gt_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.ekf_callback)

        # Initialize variables to store the ground truth and estimated positions
        self.gt_pose = None
        self.ekf_pose = None
        self.ekf_covariance = None

        # Lists to store error and uncertainty for plotting
        self.ekf_pose_x = []
        self.ekf_pose_y = []
        self.errors_x = []
        self.errors_y = []
        self.times = []
        self.uncertainties_x = []
        self.uncertainties_y = []

    def gt_callback(self, gt_msg):
        """Callback function to handle ground truth pose."""
        self.gt_pose = gt_msg.pose.pose

    def ekf_callback(self, ekf_msg):
        """Callback function to handle EKF estimated pose and covariance."""
        self.ekf_pose = ekf_msg.pose.pose
        self.ekf_covariance = ekf_msg.pose.covariance

        # Calculate the error if both gt and ekf poses are available
        if self.gt_pose and self.ekf_pose:
            self.calculate_and_store_error(ekf_msg.header.stamp)

    def calculate_and_store_error(self, timestamp):
        """Calculate and store the error between ground truth and EKF-based localization."""
        # Extract ground truth and estimated positions
        gt_pos = np.array([self.gt_pose.position.x, self.gt_pose.position.y])
        ekf_pos = np.array([self.ekf_pose.position.x, self.ekf_pose.position.y])

        # Calculate position error
        error = gt_pos - ekf_pos
        position_error = np.linalg.norm(error)

        rospy.loginfo(f"Position Error: {position_error:.3f} meters")

        # Extract uncertainty from covariance matrix (x, y variances are on diagonal: indices 0, 7)
        uncertainty_x = self.ekf_covariance[0]  # Variance in x
        uncertainty_y = self.ekf_covariance[7]  # Variance in y

        # Store the errors and uncertainties for later plotting
        self.ekf_pose_x.append(self.ekf_pose.position.x)
        self.ekf_pose_y.append(self.ekf_pose.position.y)
        self.errors_x.append(error[0])
        self.errors_y.append(error[1])
        self.times.append(timestamp.to_sec())  # Store the timestamp in seconds
        self.uncertainties_x.append(np.sqrt(uncertainty_x))  # Standard deviation in x
        self.uncertainties_y.append(np.sqrt(uncertainty_y))  # Standard deviation in y

    def plot_results(self):
        """Plot the position errors and 95% confidence intervals over time."""
        plt.figure()

        # Plot X error with 95% confidence interval
        plt.subplot(2, 1, 1)
        plt.plot(self.times, self.ekf_pose_x, label='Estimated EKF Position - Axis X')
        
        # Confidence interval: [ekf_position - 2*uncertainty, ekf_position + 2*uncertainty]
        plt.fill_between(self.times,
                        np.array(self.ekf_pose_x) - 2 * np.array(self.uncertainties_x),
                        np.array(self.ekf_pose_x) + 2 * np.array(self.uncertainties_x),
                        color='b', alpha=0.2, label='95% Confidence Interval (X)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.legend()

        # Plot Y error with 95% confidence interval
        plt.subplot(2, 1, 2)
        plt.plot(self.times, self.ekf_pose_y, label='Estimated EKF Position - Axis Y')
        
        # Confidence interval: [ekf_position - 2*uncertainty, ekf_position + 2*uncertainty]
        plt.fill_between(self.times,
                        np.array(self.ekf_pose_y) - 2 * np.array(self.uncertainties_y),
                        np.array(self.ekf_pose_y) + 2 * np.array(self.uncertainties_y),
                        color='r', alpha=0.2, label='95% Confidence Interval (Y)')
        
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.legend()

        plt.tight_layout()
        plt.show()


if __name__ == '__main__':
    try:
        error_calculator = ErrorCalculator()

        # Wait for the ROS bag to finish
        rospy.spin()

        # Plot the results after ROS bag is finished
        error_calculator.plot_results()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
