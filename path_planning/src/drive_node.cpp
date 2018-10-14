#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "tf/transform_listener.h"

float odom_x;
float odom_y;
float odom_w;

float goal_x;
float goal_y;
float goal_w;
bool stop;


void path_callback(const nav_msgs::Odometry::ConstPtr path_msg){
  float qyy = path_msg -> pose.pose.orientation.y;
  float qzz = path_msg -> pose.pose.orientation.z;

  goal_x = path_msg -> pose.pose.position.x;
  goal_y = path_msg -> pose.pose.position.y;
  goal_w = acos(1-2*pow(qyy, 2)-2*pow(qzz,2));
}

void odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg){
  float qy = odom_msg -> pose.pose.orientation.y;
  float qz = odom_msg -> pose.pose.orientation.z;

  odom_x = odom_msg -> pose.pose.position.x;
  odom_y = odom_msg -> pose.pose.position.y;
  //gives w in radians
  odom_w = acos(1-2*pow(qy,2)-2*pow(qz,2));
}

void obst_callback(const std_msgs::Bool::ConstPtr& obst_msg){
  stop = obst_msg;
}

int main(int argc, char **argv)
{
  stop = false;

  ros::init(argc, argv, "drive_node");

  ros::NodeHandle n;

  tf::TransformListener transformlistner;

  ros::Subscriber odom_sub = n.subscribe("wheel_odometry", 10, odom_callback);
  ros::Subscriber path_sub = n.subscribe("path", 10, path_callback);
  ros::Subscriber obstacle_sub = n.subscribe("obstacle", 10, obst_callback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 10); //maybe the topic has another name

  //add subscription that makes stop = true if an obstacle

  ros::Rate loop_rate(25);
  while(ros::ok()){
    geometry_msgs::Twist twist_msg;

    tf::StampedTransform transform;
    try{
      transformlistner.lookupTransform("base_link", "map", ros::Time(0), transform);
    }
    catch(tf::TransformException ex){
      ROS_WARN("%s", ex.what());
      ros::Duration(0.1).sleep();
    }


    if(stop){
      twist_msg.linear.x = 0;
      twist_msg.linear.y = 0;
      twist_msg.angular.z = 0;
    }
    else{

      float angle_to_point = atan((goal_x-odom_x)/(goal_y-odom_y));
      float distance_goal = sqrt(pow((goal_x-odom_x),2)+pow((goal_y-odom_y),2));
      ROS_INFO("angle: %f", angle_to_point-odom_w);
      ROS_INFO("dist: %f", distance_goal);

      if(distance_goal < 0.1){
        ROS_INFO("final destination %f", distance_goal);
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;
      }
      else if(abs(angle_to_point-odom_w) > 0.3){
        ROS_INFO("spinning %f", angle_to_point-odom_w);
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0.75*sign(angle_to_point-odom_w);
      }
      else{
        ROS_INFO("driving %f", distance_goal);
        float horizon = 0.3;

        twist_msg.linear.x = 0.25;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0.75*sign(angle_to_point-odom_w);
      }
    }

    //publish the twist
    twist_pub.publish(twist_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}

int sign(float x) {
  return x>0 ? 1 : -1;
}
