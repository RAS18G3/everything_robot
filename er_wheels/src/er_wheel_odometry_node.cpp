#include "er_wheel_odometry_node.h"


WheelOdometryNode::WheelOdometryNode() : ticks_per_rev_(3591.84/4), wheel_radius_(0.04), base_radius_(0.085) {
  angular_velocity_[MotorSide::Left] = 0;
  angular_velocity_[MotorSide::Right] = 0;
  // note messages are initialized with 0, so intiially the /base_frame will align with the /map frame
}

WheelOdometryNode::~WheelOdometryNode() {

}

void WheelOdometryNode::run_node() {
  ros::NodeHandle n;

  last_odometry_msg_.header.stamp = ros::Time::now();
  last_odometry_msg_.header.frame_id = "/map";
  last_odometry_msg_.child_frame_id = "/base_frame";

  odometry_pub_ = n.advertise<nav_msgs::Odometry>("/wheel_odometry", 1);

  std::function<void(const phidgets::motor_encoder::ConstPtr&)> left_cb = std::bind(&WheelOdometryNode::encoder_callback, this, MotorSide::Left, std::placeholders::_1);
  std::function<void(const phidgets::motor_encoder::ConstPtr&)> right_cb = std::bind(&WheelOdometryNode::encoder_callback, this, MotorSide::Right, std::placeholders::_1);

  ros::Subscriber encoder_sub_left = n.subscribe<phidgets::motor_encoder>("/motor_name_left/encoder", 1, left_cb);
  ros::Subscriber encoder_sub_right = n.subscribe<phidgets::motor_encoder>("/motor_name_right/encoder", 1, right_cb);

  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    // get most up-to-date encoder and twist readings
    ros::spinOnce();

    loop_rate.sleep();

  }
}

void WheelOdometryNode::encoder_callback(const enum MotorSide motor_side, const phidgets::motor_encoder::ConstPtr& encoder_msg) {
  if (last_encoder_reading_[motor_side] == -1) {
    last_encoder_reading_[motor_side] = encoder_msg->count;
    last_encoder_reading_time_[motor_side] = encoder_msg->header.stamp;
  }
  else {
    double delta_t = (encoder_msg->header.stamp-last_encoder_reading_time_[motor_side]).toSec();
    angular_velocity_[motor_side] = 2 * M_PI * (double)(encoder_msg->count - last_encoder_reading_[motor_side]) / (delta_t * ticks_per_rev_);
    last_encoder_reading_[motor_side] = encoder_msg->count;
    last_encoder_reading_time_[motor_side] = encoder_msg->header.stamp;
  }
  update_odometry();
}

void WheelOdometryNode::update_odometry() {
  // check e.g. http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf for the formulas
  nav_msgs::Odometry odometry_msg;
  const double wheel_circumference = 2*M_PI*wheel_radius_;

  odometry_msg.header.frame_id = "/map";
  odometry_msg.child_frame_id = "/base_frame";
  odometry_msg.header.stamp = ros::Time::now();
  odometry_msg.twist.twist.angular.z = (angular_velocity_[MotorSide::Right] - angular_velocity_[MotorSide::Left]) * wheel_radius_ / base_radius_;
  odometry_msg.twist.twist.linear.x = (angular_velocity_[MotorSide::Right] + angular_velocity_[MotorSide::Left]) / 2;

  // how much time has passed since the last last and current update
  const double dt = (odometry_msg.header.stamp - last_odometry_msg_.header.stamp).toSec();

  // since we dont know when exactly between the last and current the velocity has changed, we take the average of the two measurements
  const double v_avg = (odometry_msg.twist.twist.linear.x + last_odometry_msg_.twist.twist.linear.x) / 2;
  const double omega_avg = (odometry_msg.twist.twist.linear.x + last_odometry_msg_.twist.twist.linear.x) / 2;
  const double delta_omega = omega_avg * dt;

  // TODO: any nicer way of doing that??
  tf2::Quaternion relative_rotation;
  relative_rotation.setRPY(0, 0, delta_omega);
  tf2::Quaternion current_rotation;
  tf2::fromMsg(last_odometry_msg_.pose.pose.orientation, current_rotation);
  tf2::Quaternion new_rotation = (relative_rotation * current_rotation).normalize();
  odometry_msg.pose.pose.orientation = tf2::toMsg(new_rotation);

  // find radius and center of the circle trajectory that the robot has been following the past dt
  const double r = v_avg / omega_avg; // TODO: check for omega_avg == 0? required?
  const double theta = new_rotation.getAngle();
  const double icc_x = last_odometry_msg_.pose.pose.position.x - r * sin(theta);
  const double icc_y = last_odometry_msg_.pose.pose.position.y + r * cos(theta);
  const double delta_x = last_odometry_msg_.pose.pose.position.x - icc_x;
  const double delta_y = last_odometry_msg_.pose.pose.position.y - icc_y;

  odometry_msg.pose.pose.position.x = icc_x + cos(delta_omega) * delta_x - sin(delta_omega * delta_y);
  odometry_msg.pose.pose.position.y = icc_y + sin(delta_omega) * delta_x + cos(delta_omega * delta_y);

  last_odometry_msg_ = odometry_msg;
  odometry_pub_.publish(odometry_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_wheel_odometry_node");

  WheelOdometryNode wheel_odometry_node;

  wheel_odometry_node.run_node();

  return 0;
}
