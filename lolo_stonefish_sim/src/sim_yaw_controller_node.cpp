#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

class LoloYawController {
private:

    // tf listeners
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    // parameters
    double max_thruster_angle;
    std::string frame_id;

    // Integrator.
    double integrator = 0.0;

    // subscribers
    ros::Subscriber setpoint_sub;

    // publishers
    ros::Publisher rudder_pub;

public:

    LoloYawController(ros::NodeHandle& nh) : tfListener(tfBuffer)
    {
        ROS_INFO("Initializing yaw controller...");
        ros::NodeHandle pn("~");
        pn.param<double>("max_thruster_angle", max_thruster_angle, 0.6);
        pn.param<std::string>("frame_id", frame_id, "base_link");

        rudder_pub = nh.advertise<std_msgs::Float32>("core/rudder_cmd", 1000);
        setpoint_sub = nh.subscribe("ctrl/yaw_setpoint", 1000, &LoloYawController::yaw_callback, this);
        ROS_INFO("Done initializing yaw controller...");
    }

    void yaw_callback(const std_msgs::Float64& yaw)
    {
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer.lookupTransform("map", frame_id, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_INFO("Could not get map->%s transform...", frame_id.c_str());
            return;
        }

        Eigen::Affine3d transform_matrix = tf2::transformToEigen(transformStamped);

        Eigen::Vector3d euler_angles = transform_matrix.rotation().eulerAngles(2, 1, 0);

        Eigen::Vector2d goal_dir(cos(yaw.data), sin(yaw.data));
        Eigen::Vector2d vehicle_frame =
            transform_matrix.rotation().topLeftCorner<2, 2>().transpose() *
            goal_dir;

        double yaw_offset = atan2(vehicle_frame[1], vehicle_frame[0]);
        // FIXME(aldoteran): error over node frequency?
        integrator = integrator + (yaw_offset / 10);

        //ROS_INFO("Yaw setpoint: %f, current angle: %f, offset: %f",
                 //180. / M_PI * yaw.data, 180. / M_PI * euler_angles(0),
                 //180. / M_PI * yaw_offset);

        // 0.1 radians per PI radian offset
        std_msgs::Float32 angles;
        //angles.data = std::min(max_thruster_angle,
                               //std::max(-max_thruster_angle, -1. * yaw_offset));
        angles.data =
            std::min(max_thruster_angle,
                     std::max(-max_thruster_angle,
                              -1. * yaw_offset - 0.1 * integrator));
        rudder_pub.publish(angles);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sim_yaw_controller_node");

    ros::NodeHandle nh;

    LoloYawController controller(nh);

    ros::spin();

    return 0;
}
