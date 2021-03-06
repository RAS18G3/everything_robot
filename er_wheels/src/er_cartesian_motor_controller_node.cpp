#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include <cmath>

// TODO: use rosparam to set these
const double b = 0.085; // distance from wheel to base
const double r = 0.049; // radius of the wheels

const double wheel_circumference = 2*M_PI*r;

// two publishers, which are global, s.t. we can publish inside the twistCallback
ros::Publisher angular_velocity_left_pub;
ros::Publisher angular_velocity_right_pub;

void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  double omega_left = (twist_msg->linear.x - b * twist_msg->angular.z)/r;
  double omega_right = (twist_msg->linear.x + b * twist_msg->angular.z)/r;

  std_msgs::Float64 angular_velocity_left_msg;
  angular_velocity_left_msg.data = omega_left;

  std_msgs::Float64 angular_velocity_right_msg;
  angular_velocity_right_msg.data = omega_right;

  angular_velocity_left_pub.publish(angular_velocity_left_msg);
  angular_velocity_right_pub.publish(angular_velocity_right_msg);

  ROS_DEBUG("omega_left: [%f]", omega_left);
  ROS_DEBUG("omega_right: [%f]", omega_right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_cartesian_motor_controller_node");

  ros::NodeHandle n;

  std::string node_name = ros::this_node::getName();
  std::string left_motor_name, right_motor_name;
  n.param<std::string>("left_motor_name", left_motor_name, "left_motor");
  n.param<std::string>("right_motor_name", right_motor_name, "right_motor");

  angular_velocity_left_pub = n.advertise<std_msgs::Float64>(left_motor_name + "/angular_velocity", 1);
  angular_velocity_right_pub = n.advertise<std_msgs::Float64>(right_motor_name + "/angular_velocity", 1);
  ros::Subscriber twist_sub = n.subscribe(node_name + "/twist", 1, twistCallback);

  while (ros::ok())
  {
    // get most up-to-date encoder and twist readings, publishing is done inside the callback
    ros::spin();
  }


  return 0;
}
