#ifndef ER_WHEEL_ODOMETERY_NODE_H
#define ER_WHEEL_ODOMETERY_NODE_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "phidgets/motor_encoder.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <cmath>
#include <functional>
#include <map>

class WheelOdometryNode {
public:
  WheelOdometryNode();
  ~WheelOdometryNode();

  void run_node();
  void update_odometry();

private:
  enum MotorSide {Left, Right};
  void encoder_callback(const enum MotorSide motor_side, const phidgets::motor_encoder::ConstPtr& encoder_msg);
  std::map<enum MotorSide, double> angular_velocity_;
  std::map<enum MotorSide, double> last_encoder_reading_;
  std::map<enum MotorSide, ros::Time> last_encoder_reading_time_;

  const double ticks_per_rev_;
  const double wheel_radius_;
  const double base_radius_; // distance from center to wheel
  nav_msgs::Odometry last_odometry_msg_;

  ros::Publisher odometry_pub_;

  double yaw_;
};

#endif
