#include "ros/ros.h"
#include "nav_msgs/Odometry.h"


int main(int argc, char **argv)
{
float x = -2;
float y = -1;


  ros::init(argc, argv, "path_node");

  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Odometry>("path", 10);
  ros::Rate loop_rate(10);


  while(ros::ok()){
    nav_msgs::Odometry msg;

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;

    path_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
