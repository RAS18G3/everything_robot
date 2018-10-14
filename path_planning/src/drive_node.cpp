#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

double odom_x;
double odom_y;
double odom_w;

double goal_x;
double goal_y;
double goal_w;
bool stop;


void path_callback(const nav_msgs::Odometry::ConstPtr path_msg){
  double qyy = path_msg -> pose.pose.orientation.y;
  double qzz = path_msg -> pose.pose.orientation.z;

  goal_x = path_msg -> pose.pose.position.x;
  goal_y = path_msg -> pose.pose.position.y;
  goal_w = acos(1-2*pow(qyy, 2)-2*pow(qzz,2));
}

void odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg){
  double qx = odom_msg -> pose.pose.orientation.x;
  double qy = odom_msg -> pose.pose.orientation.y;
  double qz = odom_msg -> pose.pose.orientation.z;
  double qw = odom_msg -> pose.pose.orientation.w;

  odom_x = odom_msg -> pose.pose.position.x;
  odom_y = odom_msg -> pose.pose.position.y;
  //gives w in radians
  //odom_w = acos(1-2*pow(qy,2)-2*pow(qz,2));
  double roll, pitch;
  tf2::Quaternion quaternion;
  tf2::fromMsg(odom_msg->pose.pose.orientation, quaternion);
  tf2::Matrix3x3 rotation_matrix(quaternion);
  rotation_matrix.getRPY(roll, pitch, odom_w);

}

int sign(double x) {
  return x>0 ? 1 : -1;
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
      ROS_INFO("x %f", odom_x);
      ROS_INFO("y %f \n", odom_y);
      ROS_INFO("w %f", odom_w*57.32);

      double angle_to_point = atan2((goal_y-odom_y),(goal_x-odom_x));
      //double angle_to_point = atan((goal_y-odom_y)/(goal_x-odom_x));
      double distance_goal = sqrt(pow((goal_x-odom_x),2)+pow((goal_y-odom_y),2));
      double diff = angle_to_point-odom_w;

      if (diff > M_PI)
        diff -= 2*M_PI;
      else if(diff < - M_PI)
        diff += 2*M_PI;

      if(distance_goal < 0.1){
        ROS_INFO("final destination %f", distance_goal);
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;
      }
      else if(std::abs(diff) > 0.2){
        ROS_INFO("spinning");
        ROS_INFO("a2p %f", angle_to_point*57.32);
        ROS_INFO("diff %f", (diff)*57.32);
        ROS_INFO("abs %f ", std::abs(diff));
        ROS_INFO("distance %f \n", distance_goal);
        //ROS_INFO("angular velocity %f \n", 0.75*sign(angle_to_point-odom_w));

        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;


        twist_msg.angular.z = 0.75*sign(diff);
      }
      else{
        ROS_INFO("Driving");
        ROS_INFO("a2p %f", angle_to_point*57.32);
        ROS_INFO("distance %f ", distance_goal);
        ROS_INFO("abs %f ", std::abs(diff));
        ROS_INFO("diff %f \n", (diff)*57.32);

        twist_msg.linear.x = 0.25;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 1*(diff);
      }
    }

    //publish the twist
    twist_pub.publish(twist_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

}
