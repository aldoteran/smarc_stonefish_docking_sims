#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float64
from smarc_msgs.msg import ThrusterRPM
from nav_msgs.msg import Odometry

class LoloController():

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
        self.elevator_ctrl = Float32()

        # Elevator limits [rad?] [min, max].
        self.elevator_lims = [-0.6, 0.6]

        # Publishers.
        self.thruster_stbd_pub = rospy.Publisher(rospy.get_param("/thruster_stbd_topic"),
                                            ThrusterRPM, queue_size=1)
        self.thruster_port_pub = rospy.Publisher(rospy.get_param("/thruster_port_topic"),
                                            ThrusterRPM, queue_size=1)
        self.heading_pub = rospy.Publisher(rospy.get_param("/heading_setpoint_topic"),
                                          Float64, queue_size=1)
        self.elevator_pub = rospy.Publisher(rospy.get_param("/elevator_cmd_topic"),
                                            Float32, queue_size=1)
        # VBS publishers.
        self.vbs_front_stbd_pub = rospy.Publisher(rospy.get_param("/vbs_front_stbd_topic"),
                                            Float64, queue_size=0)
        self.vbs_front_port_pub = rospy.Publisher(rospy.get_param("/vbs_front_port_topic"),
                                            Float64, queue_size=0)
        self.vbs_back_stbd_pub = rospy.Publisher(rospy.get_param("/vbs_back_stbd_topic"),
                                            Float64, queue_size=0)
        self.vbs_back_port_pub = rospy.Publisher(rospy.get_param("/vbs_back_port_topic"),
                                            Float64, queue_size=0)

    def publish_navigation(self):
        """
        Publish thrusters' RPM and heading setpoint.
        """
        rpms = ThrusterRPM()
        rpms.rpm = self.tgt_rpm
        self.thruster_stbd_pub.publish(rpms)
        self.thruster_port_pub.publish(rpms)

        heading = Float64()
        heading.data = self.tgt_heading
        self.heading_pub.publish(heading)


    def fill_vbs(self):
        """
        Fill VBS to preset capacity to help lolo dive.
        """
        vbs_vol = Float64()
        vbs_vol.data = self.tgt_vbs
        self.vbs_front_stbd_pub.publish(vbs_vol)
        self.vbs_front_port_pub.publish(vbs_vol)
        self.vbs_back_stbd_pub.publish(vbs_vol)
        self.vbs_back_port_pub.publish(vbs_vol)

    def start_listener(self):
        rospy.Subscriber(rospy.get_param("/lolo_odom_topic"), Odometry,
                         self.odometry_callback, queue_size=1)

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
        if ctrl > self.elevator_lims[1]:
            ctrl = self.elevator_lims[1]
        elif ctrl < self.elevator_lims[0]:
            ctrl = self.elevator_lims[0]
        self.elevator_ctrl.data = ctrl


        self.prev_error = error

        self.elevator_pub.publish(self.elevator_ctrl)


if __name__ == "__main__":
    """
    Run the elevator control node.
    """
    rospy.init_node("lolo_target_controller")

    controller = LoloController()
    controller.start_listener()

    rate = rospy.Rate(controller.node_freq)

    while not rospy.is_shutdown():
        controller.publish_navigation()
        controller.fill_vbs()
        rate.sleep()

