#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from smarc_msgs.msg import ThrusterRPM
from sam_msgs.msg import ThrusterAngles, PercentStamped
from nav_msgs.msg import Odometry

class SamController():

    def __init__(self):

        self.cur_depth = 0.0

        # Target RPM.
        self.tgt_rpm = rospy.get_param("/target_rpm")
        # Target heading angle in [rads], ENU convention.
        self.tgt_heading = rospy.get_param("/target_heading")
        # Target depth in [m].
        self.tgt_depth = rospy.get_param("/target_depth")
        # Target VBS capaticy.
        self.tgt_vbs = rospy.get_param("/target_vbs")
        # Target LCG position.
        self.tgt_lcg = rospy.get_param("/target_lcg")

        # Propotional coefficient.
        self.p_ = rospy.get_param("/p_coefficient")
        # Integral coefficient.
        self.i_ = rospy.get_param("/i_coefficient")
        # Derivative coefficient.
        self.d_ = rospy.get_param("/d_coefficient")
        # Integrator.
        self.integrator = 0.0
        # Previous error.
        self.prev_error = 0.0
        # Node frequency (max 10Hz due to odom feedback).
        self.node_freq = 10.0
        # Init ctrl message.
        self.thrust_vector_ctrl = ThrusterAngles()

        # Thrust vector limits [rads] [min, max].
        self.thrust_vector_lims = [-0.12, 0.12]

        # Publishers.
        self.thruster1_pub = rospy.Publisher(rospy.get_param("/thruster1_topic"),
                                            ThrusterRPM, queue_size=1)
        self.thruster2_pub = rospy.Publisher(rospy.get_param("/thruster2_topic"),
                                            ThrusterRPM, queue_size=1)
        self.heading_pub = rospy.Publisher(rospy.get_param("/heading_setpoint_topic"),
                                          Float64, queue_size=1)
        self.thrust_vector_pub = rospy.Publisher(rospy.get_param("/thrust_vector_topic"),
                                                 ThrusterAngles, queue_size=1)
        # VBS and LCG publishers.
        self.vbs_pub = rospy.Publisher(rospy.get_param("/vbs_cmd_topic"),
                                            PercentStamped, queue_size=0)
        self.lcg_pub = rospy.Publisher(rospy.get_param("/lcg_cmd_topic"),
                                            PercentStamped, queue_size=0)

    def publish_navigation(self):
        """
        Publish thrusters' RPM and heading setpoint.
        """
        rpms = ThrusterRPM()
        rpms.rpm = self.tgt_rpm
        self.thruster1_pub.publish(rpms)
        self.thruster2_pub.publish(rpms)

        heading = Float64()
        heading.data = self.tgt_heading
        self.heading_pub.publish(heading)


    def trim(self):
        """
        Fill VBS to preset capacity to help sam dive and
        set LCG to max to avoid pitching.
        """
        vbs_vol = PercentStamped()
        vbs_vol.value = self.tgt_vbs
        self.vbs_pub.publish(vbs_vol)

        lcg_pos = PercentStamped()
        lcg_pos.value = self.tgt_lcg
        self.lcg_pub.publish(lcg_pos)

    def start_listener(self):
        rospy.Subscriber(rospy.get_param("/odom_topic"), Odometry,
                         self.odometry_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("/horizontal_thrust_topic"), ThrusterAngles,
                         self.horizontal_angle_callback, queue_size=1)

    def odometry_callback(self, msg):
        """
        We're gonna send a ctrl input every time this callback gets
        trigged.
        """

        error = self.tgt_depth - msg.pose.pose.position.z

        self.integrator += error / self.node_freq
        slope = (error - self.prev_error) * self.node_freq

        # Calculate PID control input.
        ctrl = -(self.p_*error + self.i_*self.integrator + self.d_*slope)

        # print("Error:{0}".format(error))
        # print("Control:{0}".format(ctrl))

        # Make sure we're within the limits of the actuator.
        if ctrl > self.thrust_vector_lims[1]:
            ctrl = self.thrust_vector_lims[1]
        elif ctrl < self.thrust_vector_lims[0]:
            ctrl = self.thrust_vector_lims[0]
        self.thrust_vector_ctrl.thruster_vertical_radians = ctrl
        self.thrust_vector_ctrl.header.stamp = rospy.Time.now()

        self.prev_error = error

        self.thrust_vector_pub.publish(self.thrust_vector_ctrl)

    def horizontal_angle_callback(self, msg):
        """
        Stores the horizontal angle computed by the yaw controller.
        """

        self.thrust_vector_ctrl.thruster_horizontal_radians = \
            msg.thruster_horizontal_radians


if __name__ == "__main__":
    """
    Run the elevator control node.
    """
    rospy.init_node("lolo_target_controller")

    controller = SamController()
    controller.start_listener()

    rate = rospy.Rate(controller.node_freq)

    while not rospy.is_shutdown():
        controller.publish_navigation()
        controller.trim()
        rate.sleep()

