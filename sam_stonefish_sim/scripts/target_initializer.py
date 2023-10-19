#!/usr/bin/env python

import rospy
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np

class TargetInitializer():

    def __init__(self):

        # Target's initial position stddev.
        self.tgt_pos_stddev = rospy.get_param("~initial_pos_stddev")
        # Target's initial orientation stddev.
        self.tgt_rot_stddev = rospy.get_param("~initial_rot_stddev")

        # Target's GT frame name.
        self.tgt_gt_frame = rospy.get_param("~target_true_frame")
        # Target's base frame name.
        self.tgt_frame = rospy.get_param("~target_frame")
        # Target's map frame name.
        self.map_frame = rospy.get_param("~map_frame")

        # Publishers.
        self.pose_pub = rospy.Publisher(rospy.get_param("~initial_pose_topic"),
                                        PoseWithCovarianceStamped,
                                        queue_size=1, latch=True)

        # TF service.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_initial_pose(self):
        """
        Publishes a noisy initial pose by adding zero mean Gaussian noise to the
        ground truth estimate.
        """
        rate = rospy.Rate(1.0)
        # Get TF map->gt target frame.
        while not rospy.is_shutdown():
            try:
                tfm = self.tf_buffer.lookup_transform(self.map_frame, self.tgt_gt_frame,
                                                    rospy.Time())
                break
            except:
                rospy.logerr("(TargetInitializer): Could not find tfm %s to %s",
                            self.map_frame, self.tgt_gt_frame)

                rate.sleep()

        # Compute position and orientation noise from zero mean Gaussian.
        pos_noise = np.random.normal(0, self.tgt_pos_stddev)
        rot_noise = np.random.normal(0, self.tgt_rot_stddev)

        # Add noise to rotation and turn back into quats.
        rot = Rotation.from_quat([tfm.transform.rotation.x,
                                    tfm.transform.rotation.y,
                                    tfm.transform.rotation.z,
                                    tfm.transform.rotation.w])
        euler = rot.as_euler('xyz')
        euler = np.array([euler[0] + rot_noise, euler[1] + rot_noise,
                            euler[2] + rot_noise])
        rot = Rotation.from_euler('xyz', euler)
        quat = rot.as_quat()

        # Build pose message.
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = tfm.header
        pose_msg.pose.pose.position.x = tfm.transform.translation.x + pos_noise
        pose_msg.pose.pose.position.y = tfm.transform.translation.y + pos_noise
        pose_msg.pose.pose.position.z = tfm.transform.translation.z + pos_noise
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]

        # Add the diagonal covariance.
        pose_msg.pose.covariance[0] = pos_noise*pos_noise
        pose_msg.pose.covariance[7] = pos_noise*pos_noise
        pose_msg.pose.covariance[14] = pos_noise*pos_noise
        pose_msg.pose.covariance[21] = rot_noise*rot_noise
        pose_msg.pose.covariance[28] = rot_noise*rot_noise
        pose_msg.pose.covariance[35] = rot_noise*rot_noise

        self.pose_pub.publish(pose_msg)

        rospy.loginfo("(TargetInitializer): Published initial target's pose.")

if __name__ == "__main__":
    """
    Run the initializer.
    """
    rospy.init_node("target_initializer")

    tgt_init = TargetInitializer()

    tgt_init.publish_initial_pose()

    while not rospy.is_shutdown():
        rospy.spin()


