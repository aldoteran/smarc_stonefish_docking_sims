/**
 * @file UsblDriver.cpp
 * @brief Manipulates USBL measurements to fit real conditions
 * @date Apr 20, 2022
 * @author aldo terán (aldot@kth.se)
 * @author antonio terán (teran@mit.edu)
 */


#include <ros/ros.h>
#include <ros/console.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <eigen_conversions/eigen_msg.h>

#include <random>
#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#define SOUNDSPEED 1531.0 // from stonefish sim

// TODO: we could get rid of global variables by marking a class
// and having the nodehandle as a private variable that we can call
// inside the callback to getparams from the rosparam server.
std::string usbl_in_topic;
std::string usbl_out_topic;
std_msgs::Float64MultiArray usbl_msg;
std::string map_frame_id;
std::string target_usbl_frame_id;
std::string chaser_usbl_frame_id;

ros::Publisher usbl_pub;

/* ************************************************************************** */
/* ************************************************************************** */
/* ************************************************************************** */

void markerCallback(const visualization_msgs::MarkerArray &msg){
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try {
    listener.waitForTransform(map_frame_id, target_usbl_frame_id, ros::Time(0),
                              ros::Duration(0.5));
    listener.lookupTransform(map_frame_id, target_usbl_frame_id, ros::Time(0),
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Skipping USBL measurment");
    return;
  }
  // Extract rotation only
  tf::Matrix3x3 rotation = transform.getBasis();

  // Get usbl fix in cartesian
  tf::Vector3 cart(msg.markers[0].pose.position.x,
                   msg.markers[0].pose.position.y,
                   msg.markers[0].pose.position.z);

  //std::cout << "(USBLDriver) Got USBL measurement" << std::endl;
  //std::cout << cart.x() << " " << cart.y() << " " << cart.z() << std::endl;

  // Rotate the usbl fix to the world frame (ENU)
  cart = rotation * cart;

  try {
    listener.waitForTransform(map_frame_id, chaser_usbl_frame_id, ros::Time(0),
                              ros::Duration(0.5));
    listener.lookupTransform(map_frame_id, chaser_usbl_frame_id, ros::Time(0),
                             transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ROS_ERROR("Skipping USBL measurment");
    return;
  }
  // Extract rotation only
  rotation = transform.getBasis();
  // Rotate the usbl fix to the world chaser frame.
  cart = - (rotation.transpose() * cart);

  //Eigen::Matrix3d rot;
  //tf::matrixTFToEigen(rotation, rot);
  //Eigen::Quaterniond q(rot);
  //std::cout << "(USBLDriver) Rotation matrix " << std::endl;
  //std::cout << q.w() << std::endl;
  //std::cout << q.vec() << std::endl;

  //std::cout << "(USBLDriver) Rotated USBL Measurement" << std::endl;
  //std::cout << cart.x() << " " << cart.y() << " " << cart.z() << std::endl;

  // Build message.
  visualization_msgs::Marker meas;
  meas.header.frame_id = chaser_usbl_frame_id;
  meas.pose.position.x = cart.x();
  meas.pose.position.y = cart.y();
  meas.pose.position.z = cart.z();
  meas.pose.orientation.w = 1.0;
  // Define marker properties.
  meas.type = 2; // Sphere.
  meas.scale.x = 0.15;
  meas.scale.y = 0.15;
  meas.scale.z = 0.15;
  meas.color.b = 1.0;
  meas.color.a = 1.0;

  // calculate delay and wait to publish
  double range = sqrt(pow(cart.x(), 2) + pow(cart.y(), 2) + pow(cart.z(), 2));
  double delay = range / SOUNDSPEED;
  std::cout << "(USBLDriver) range and delay: " << range << "m " << delay << "s"
            << std::endl;
  ros::Duration(delay).sleep();

  // Stamp the outgoing message and publish.
  meas.header.stamp = ros::Time::now();
  usbl_pub.publish(meas);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "chaser_usbl_driver");
    ros::NodeHandle nh;

    // Get names
    nh.param<std::string>("usbl_in_topic", usbl_in_topic, "/lolo/sim/usbl");
    nh.param<std::string>("usbl_out_topic", usbl_out_topic, "/sam/sim/usbl");
    nh.param<std::string>("target_usbl_frame_id", target_usbl_frame_id,
                          "/lolo/usbl_link");
    nh.param<std::string>("chaser_usbl_frame_id", chaser_usbl_frame_id,
                          "/sam/usbl_link");
    nh.param<std::string>("map_frame_id", map_frame_id, "/map");

    usbl_pub = nh.advertise<visualization_msgs::Marker>(usbl_out_topic, 1);
    ros::Subscriber usbl_sub = nh.subscribe(usbl_in_topic, 1, markerCallback);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

  return 0;
}
