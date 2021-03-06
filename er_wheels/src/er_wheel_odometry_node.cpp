#include "er_wheel_odometry_node.h"


WheelOdometryNode::WheelOdometryNode() : ticks_per_rev_(3591.84/4), wheel_radius_(0.049), base_radius_(0.09), yaw_(0), loop_rate_(50) {
  angular_velocity_[MotorSide::Left] = 0;
  angular_velocity_[MotorSide::Right] = 0;
  // note messages are initialized with 0, so intiially the /base_frame will align with the /map frame
}

WheelOdometryNode::~WheelOdometryNode() {

}

void WheelOdometryNode::init_node() {

  // read all parameteres
  std::string node_name = ros::this_node::getName();
  std::string left_motor_name, right_motor_name;
  bool left_inverted, right_inverted;
  ros::param::param<std::string>("~left_motor_name", left_motor_name, "left_motor");
  ros::param::param<std::string>("~right_motor_name", right_motor_name, "right_motor");
  ros::param::param<bool>("~left_motor_invert", left_inverted, false);
  ros::param::param<bool>("~right_motor_invert", right_inverted, false);
  inverted_sign_[MotorSide::Left] = left_inverted ? -1 : 1;
  inverted_sign_[MotorSide::Right] = right_inverted ? -1 : 1;

  ROS_DEBUG("inverted left %d right %d", inverted_sign_[MotorSide::Left], inverted_sign_[MotorSide::Right]);

  last_odometry_msg_.header.stamp = ros::Time::now();
  last_odometry_msg_.header.frame_id = "odom";
  last_odometry_msg_.child_frame_id = "base_frame";
  last_odometry_msg_.pose.pose.orientation.w = 1;

  odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(node_name, 1);

  std::function<void(const phidgets::motor_encoder::ConstPtr&)> left_cb = std::bind(&WheelOdometryNode::encoder_callback, this, MotorSide::Left, std::placeholders::_1);
  std::function<void(const phidgets::motor_encoder::ConstPtr&)> right_cb = std::bind(&WheelOdometryNode::encoder_callback, this, MotorSide::Right, std::placeholders::_1);

  encoder_sub_left_ = nh_.subscribe<phidgets::motor_encoder>(left_motor_name + "/encoder", 1, left_cb);
  encoder_sub_right_ = nh_.subscribe<phidgets::motor_encoder>(right_motor_name + "/encoder", 1, right_cb);
}

void WheelOdometryNode::run_node() {
  init_node();

  while (ros::ok())
  {
    // get most up-to-date encoder and twist readings
    ros::spinOnce();

    // publish transform (this has to be done even if the callback is not called, so that the transform stays up-to-date)
    publish_transform();

    loop_rate_.sleep();

  }
}

void WheelOdometryNode::encoder_callback(const enum MotorSide motor_side, const phidgets::motor_encoder::ConstPtr& encoder_msg) {
  if (last_encoder_reading_[motor_side] == -1) {
    last_encoder_reading_[motor_side] = encoder_msg->count;
    last_encoder_reading_time_[motor_side] = encoder_msg->header.stamp;
  }
  else {
    double delta_t = (encoder_msg->header.stamp-last_encoder_reading_time_[motor_side]).toSec();
    angular_velocity_[motor_side] = inverted_sign_[motor_side] * 2 * M_PI * (double)(encoder_msg->count - last_encoder_reading_[motor_side]) / (delta_t * ticks_per_rev_);
    last_encoder_reading_[motor_side] = encoder_msg->count;
    last_encoder_reading_time_[motor_side] = encoder_msg->header.stamp;
  }
  update_odometry();
}

void WheelOdometryNode::update_odometry() {
  // check e.g. http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf for the formulas
  nav_msgs::Odometry odometry_msg;
  const double wheel_circumference = 2*M_PI*wheel_radius_;

  odometry_msg.header.frame_id = "odom";
  odometry_msg.child_frame_id = "base_frame";
  odometry_msg.header.stamp = ros::Time::now();
  odometry_msg.twist.twist.angular.z = (angular_velocity_[MotorSide::Right] - angular_velocity_[MotorSide::Left]) * wheel_radius_ / ( 2 * base_radius_ );
  odometry_msg.twist.twist.linear.x = (angular_velocity_[MotorSide::Right] + angular_velocity_[MotorSide::Left]) * wheel_radius_ / 2;

  // how much time has passed since the last last and current update
  const double dt = (odometry_msg.header.stamp - last_odometry_msg_.header.stamp).toSec();

  // since we dont know when exactly between the last and current the velocity has changed, we take the average of the two measurements
  // because of this it is important, that we actually publish messages frequently, even if we are not moving at all
  // otherwise we would suddenly jump because of the large dt
  const double v_avg = (odometry_msg.twist.twist.linear.x + last_odometry_msg_.twist.twist.linear.x) / 2;
  double omega_avg = (odometry_msg.twist.twist.angular.z + last_odometry_msg_.twist.twist.angular.z) / 2;
  const double delta_omega = omega_avg * dt;

  // TODO: any nicer way of doing that, using the old message is super annoying because of quaternions...?
  // make sure yaw_ is in [0,2pi)
  yaw_ = yaw_ + delta_omega;
  yaw_ = yaw_ >= 2 * M_PI ? yaw_ - 2 * M_PI : yaw_;
  yaw_ = yaw_ < 0 ? yaw_ + 2 * M_PI : yaw_;
  tf2::Quaternion new_rotation;
  new_rotation.setRPY(0, 0, yaw_);
  odometry_msg.pose.pose.orientation = tf2::toMsg(new_rotation);

  // find radius and center of the circle trajectory that the robot has been following the past dt
  // different formulas when omega_avg is very small to prevent numerical issues -> infinite circle / driving in a straight line
  if(omega_avg <= 1e-6) {
    odometry_msg.pose.pose.position.x = last_odometry_msg_.pose.pose.position.x + cos(yaw_) * v_avg * dt;
    odometry_msg.pose.pose.position.y = last_odometry_msg_.pose.pose.position.y + sin(yaw_) * v_avg * dt;
  }
  else {
    const double r = v_avg / omega_avg; // TODO: check for omega_avg == 0? required?
    const double icc_x = last_odometry_msg_.pose.pose.position.x - r * sin(yaw_);
    const double icc_y = last_odometry_msg_.pose.pose.position.y + r * cos(yaw_);
    const double delta_x = last_odometry_msg_.pose.pose.position.x - icc_x;
    const double delta_y = last_odometry_msg_.pose.pose.position.y - icc_y;

    ROS_DEBUG("delta_omega: %f", delta_omega);
    ROS_DEBUG("delta_x: %f", delta_x);
    ROS_DEBUG("delta_y: %f", delta_y);
    ROS_DEBUG("r %f", r);
    ROS_DEBUG("yaw %f", yaw_);
    ROS_DEBUG("icc_x %f", icc_x);
    ROS_DEBUG("icc_y %f", icc_y);

    odometry_msg.pose.pose.position.x = icc_x + cos(delta_omega) * delta_x - sin(delta_omega) * delta_y;
    odometry_msg.pose.pose.position.y = icc_y + sin(delta_omega) * delta_x + cos(delta_omega) * delta_y;
  }


  last_odometry_msg_ = odometry_msg;
}

void WheelOdometryNode::publish_transform()
{
  geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "odom";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = last_odometry_msg_.pose.pose.position.x;
  transform_stamped.transform.translation.y = last_odometry_msg_.pose.pose.position.y;
  transform_stamped.transform.rotation = last_odometry_msg_.pose.pose.orientation;

  transform_broadcaster_.sendTransform(transform_stamped);

  odometry_pub_.publish(last_odometry_msg_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_wheel_odometry_node");

  WheelOdometryNode wheel_odometry_node;

  wheel_odometry_node.run_node();

  return 0;
}
