#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
import numpy as np

class DataSaver:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('ekf_error_saver_node')
        rospy.set_param('/use_sim_time', True)
        
        # Subscribe to ground truth and EKF estimated pose topics
        rospy.Subscriber('/robot_pose_with_covariance_ekf', PoseWithCovarianceStamped, self.ekf_transformed_callback)
        rospy.Subscriber('/gtInfo', PoseWithCovarianceStamped, self.gt_callback)

        # Publisher for the Euclidean error
        self.error_publisher = rospy.Publisher('/ekf_error', Float64, queue_size=10)

        # Initialize variables to store error data
        self.errors = []
        self.times = []
        
        self.initial_time = None
        self.ekf_msgs = []  # List to store EKF messages

        # Filepath to save the data
        self.filepath = '/home/raquel/catkin_ws/src/turtlebot3_datasets/data/ekf_gt_error_data.npz'

        # Register the shutdown hook to save data when the node is terminated
        rospy.on_shutdown(self.save_data)

        rospy.loginfo("EKF Error Saver node initialized. Waiting for data...")
    
    def ekf_transformed_callback(self, ekf_msg):
        """Callback function to handle EKF estimated pose."""
        # Store each EKF message for future comparison
        self.ekf_msgs.append(ekf_msg)

    def gt_callback(self, gt_msg):
        """Callback function to handle ground truth pose."""
        # Ensure there are EKF messages to compare with
        if self.ekf_msgs:
            closest_ekf_msg = self.find_closest_msg(self.ekf_msgs, gt_msg.header.stamp)
            self.calculate_and_store_error(gt_msg, closest_ekf_msg)

    def find_closest_msg(self, msg_list, target_time):
        """Find the message with the closest timestamp to the target time."""
        return min(msg_list, key=lambda msg: abs((msg.header.stamp - target_time).to_sec()))

    def calculate_and_store_error(self, gt_msg, ekf_msg):
        """Calculate the Euclidean error and store it."""
        # Extract positions from the messages
        gt_position = gt_msg.pose.pose.position
        ekf_position = ekf_msg.pose.pose.position

        # Calculate the Euclidean distance between the positions
        error = np.sqrt((gt_position.x - ekf_position.x)**2 +
                        (gt_position.y - ekf_position.y)**2)

        # Get the current time (from the ground truth message)
        current_time = gt_msg.header.stamp.to_sec()

        # Set the initial time on the first message
        if self.initial_time is None:
            self.initial_time = current_time

        # Store the relative time and the error
        relative_time = current_time - self.initial_time
        self.errors.append(error)
        self.times.append(relative_time)

        # Publish the error to the /ekf_error topic
        self.error_publisher.publish(error)

        rospy.loginfo(f"Calculated error: {error} at time: {relative_time}")

    def save_data(self):
        """Save the collected data to a file."""
        rospy.loginfo("Attempting to save error data...")

        if self.times:  # Check if there's data to save
            # Convert lists to numpy arrays and save them to file
            np.savez(self.filepath,
                     times=np.array(self.times),
                     errors=np.array(self.errors))
            rospy.loginfo(f"Error data saved to {self.filepath}")
        else:
            rospy.loginfo("No data to save.")

if __name__ == '__main__':
    try:
        data_saver = DataSaver()

        # Wait for ROS messages to process
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
