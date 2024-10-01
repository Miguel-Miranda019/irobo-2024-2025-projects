#!/usr/bin/env python3

import threading
import rospy
import tf2_ros
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage

class GroundTruthPublisher:
    def __init__(self):
        rospy.init_node('ground_truth_publisher')
        rospy.set_param('/use_sim_time', True)
        
        # Publisher for ground truth poses
        self.publisher = rospy.Publisher('/gtInfo', PoseWithCovarianceStamped, queue_size=50)
        
        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Thread lock
        self.lock = threading.Lock()
        
        # Last publish time
        self.last_publish_time = rospy.Time(0)
        
        # Subscriber
        rospy.Subscriber('/tf', TFMessage, self.tf_callback)
    
    def publish_pose(self, msg):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "mocap"
        pose.header.stamp = msg.header.stamp
        pose.pose.pose.position = msg.transform.translation
        pose.pose.pose.orientation = msg.transform.rotation
        
        self.publisher.publish(pose)
        rospy.loginfo("Published ground truth info.")
    
    def tf_callback(self, tf_msg):
        try:
            for transform in tf_msg.transforms:
                if transform.child_frame_id == 'mocap_laser_link':
                    current_time = rospy.Time.now()
                    time_diff = current_time - self.last_publish_time
                    
                    if time_diff.to_sec() >= 1:  # Publish every second
                        self.publish_pose(transform)
                        self.last_publish_time = current_time
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn("Extrapolation error in tf_callback: %s", str(e))
        except Exception as e:
            rospy.logwarn("Error in tf_callback: %s", str(e))

if __name__ == '__main__':
    try:
        gt_publisher = GroundTruthPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")
