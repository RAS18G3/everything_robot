#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

float x;
float y;
int i = 0;
int num_points = 3;

void new_point_callback(const std_msgs::Bool::ConstPtr& new_point_msg){
  if(i < num_points){
    x = x_array[i];
    y = y_array[i];
    i++;
  }
}


int main(int argc, char **argv)
{
float x_array[num_points] = {1, 0, -1.5};
float y_array[num_points] = {1, 0, 1};


  ros::init(argc, argv, "path_node");

  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Odometry>("path", 10);
  ros::Subscriber new_point_sub = n.subscribe("new_point", 10, new_point_callback);
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
