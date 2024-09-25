#!/usr/bin/env python3
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovariance, Pose, Point, Quaternion

# Initialize the node
rospy.init_node('gt_info_publisher')

# Publisher for the PoseWithCovariance message
pub = rospy.Publisher('/gtInfo', PoseWithCovariance, queue_size=10)

# TF2 Buffer and Listener to retrieve transform data
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Define a function to get transform and publish PoseWithCovariance
def publish_gt_info(event):
    try:
        # Get the transform between 'mocap' and 'mocap_laser_link'
        transform = tf_buffer.lookup_transform('mocap', 'mocap_laser_link', rospy.Time(0), rospy.Duration(1.0))
        
        # Create a PoseWithCovariance message
        pose_cov_msg = PoseWithCovariance()
        
        # Set the position from the transform
        pose_cov_msg.pose.position = Point(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        )
        
        # Set the orientation from the transform
        pose_cov_msg.pose.orientation = Quaternion(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )
        
        # Covariance matrix (set it to identity or adjust based on your application)
        pose_cov_msg.covariance = [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.1
        ]
        
        pub.publish(pose_cov_msg)
        rospy.loginfo("Published ground truth info.")

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Could not get transform: %s", str(e))

rospy.Timer(rospy.Duration(1), publish_gt_info)
rospy.spin()
