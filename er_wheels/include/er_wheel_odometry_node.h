#ifndef ER_WHEEL_ODOMETERY_NODE_H
#define ER_WHEEL_ODOMETERY_NODE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "phidgets/motor_encoder.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"

#include <cmath>
#include <functional>
#include <map>

class WheelOdometryNode {
public:
  WheelOdometryNode();
  ~WheelOdometryNode();

  void run_node();

private:
  enum MotorSide {Left, Right};
  void encoder_callback(const enum MotorSide motor_side, const phidgets::motor_encoder::ConstPtr& encoder_msg);
  void init_node();
  void update_odometry();
  void publish_transform();
  std::map<enum MotorSide, double> angular_velocity_;
  std::map<enum MotorSide, double> last_encoder_reading_;
  std::map<enum MotorSide, ros::Time> last_encoder_reading_time_;
  std::map<enum MotorSide, int> inverted_sign_;

  const double ticks_per_rev_;
  const double wheel_radius_;
  const double base_radius_; // distance from center to wheel
  nav_msgs::Odometry last_odometry_msg_;

  ros::NodeHandle nh_;
  ros::Publisher odometry_pub_;
  ros::Subscriber encoder_sub_left_;
  ros::Subscriber encoder_sub_right_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  ros::Rate loop_rate_;

  double yaw_;
};

#endif
