#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


int main(int argc, char **argv)
{
float x = 0;
float y = 0;
float w = 0;


  ros::init(argc, argv, "send_odom");

  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Rate loop_rate(10);

  while(ros::ok()){
    nav_msgs::Odometry msg;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;

    odom_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
