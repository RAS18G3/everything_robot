#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Path.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include "tf/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <er_planning/PathAction.h>
#include <actionlib/server/simple_action_server.h>

double odom_x;
double odom_y;
double odom_w;

double goal_x;
double goal_y;
std::vector<double> goal_xarr;
std::vector<double> goal_yarr;

typedef actionlib::SimpleActionServer<er_planning::PathAction> Server;

bool stop;
bool forward;
bool near_end;

void find_goto_point(double horizon, int p){
  if(near_end == true){
    goal_x = goal_xarr.at(1);
    goal_y = goal_yarr.at(1);
  }
else if(goal_xarr.size() == 0){
  goal_x = odom_x;
  goal_y = odom_y;
  ROS_INFO("no goal");
}
else if(goal_xarr.size() == 1){
  goal_x = goal_xarr.at(0);
  goal_y = goal_yarr.at(0);
  ROS_INFO("one point");
}
else{
     double ABx = goal_xarr.at(p+1)-goal_xarr.at(p);
     double ABy = goal_yarr.at(p+1)-goal_yarr.at(p);
     double APx = odom_x-goal_xarr.at(p);
     double APy = odom_y-goal_yarr.at(p);
     double lengthSqrAB = pow(ABx,2)+pow(ABy,2);
     double t = (APx*ABx+APy*ABy)/lengthSqrAB;

     double nearest_x_online = goal_xarr.at(p) + t*ABx;
     double nearest_y_online = goal_yarr.at(p) + t*ABy;

     if(pow(nearest_x_online-odom_x,2)+pow(nearest_y_online-odom_y,2) > pow(horizon,2)){
       goal_x = nearest_x_online;
       goal_y = nearest_y_online;
     }

     else if(nearest_x_online==odom_x && nearest_y_online==odom_y){
       double ang = atan2(goal_yarr.at(p)-nearest_y_online, goal_xarr.at(p)-nearest_x_online);
       goal_x = nearest_x_online + horizon*cos(ang);
       goal_y = nearest_y_online + horizon*sin(ang);
     }

    else{
     double ang = atan2(goal_yarr.at(p+1)-goal_yarr.at(p), goal_xarr.at(p+1)-goal_xarr.at(p));
     double A_nearest_len = sqrt(pow(nearest_x_online-odom_x,2)+pow(nearest_y_online-odom_y,2));
     double length_plus = sqrt(pow(horizon,2)-pow(A_nearest_len,2));
     double x1 = nearest_x_online + length_plus*cos(ang);
     double x2 = nearest_x_online - length_plus*cos(ang);
     double y1 = nearest_y_online + length_plus*sin(ang);
     double y2 = nearest_y_online - length_plus*sin(ang);

     if(sqrt(pow(goal_xarr.at(p+1)-x1,2)+pow(goal_yarr.at(p+1)-y1,2)) < sqrt(pow(goal_xarr.at(p+1)-x2,2)+pow(goal_yarr.at(p+1)-y2,2))){
       goal_x = x1;
       goal_y = y1;
     }
     else{
       goal_x = x2;
       goal_y = y2;
     }
   }
  }
  ROS_INFO_STREAM("goal endpoint " << goal_xarr[1] << " " << goal_yarr[1]);


  ROS_INFO("goal x %f", goal_x);
  ROS_INFO("goal y %f", goal_y);
}

void odom_callback(const nav_msgs::Odometry::ConstPtr odom_msg){

  odom_x = odom_msg -> pose.pose.position.x;
  odom_y = odom_msg -> pose.pose.position.y;


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
  stop = obst_msg->data;
}

void execute(const er_planning::PathGoal::ConstPtr& goal, Server* as){
  std::vector<float> path = goal -> Path;
  bool last_was_y = true;
  for(int i = 0; i < path.size(); i++){
    if(last_was_y){
      goal_yarr.push_back(path.at(i));
      last_was_y = false;
  }
    else{
      goal_xarr.push_back(path.at(i));
      last_was_y = true;
    }
  }

  stop = false;
  near_end = false;
  double horizon = 0.2;
  int p = 0;

  ros::NodeHandle n;

  ros::Subscriber odom_sub = n.subscribe("wheel_odometry", 10, odom_callback);
  ros::Subscriber obstacle_sub = n.subscribe("obstacle", 10, obst_callback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 10);

  ros::Rate loop_rate(25);
  geometry_msgs::Twist twist_msg;
  while(ros::ok()){
    find_goto_point(horizon, p);

    if(stop && forward){
      ROS_INFO("stop");
      twist_msg.linear.x = 0;
      twist_msg.linear.y = 0;
      twist_msg.angular.z = 0;
    }
    else{
      ROS_INFO("x %f", odom_x);
      ROS_INFO("y %f \n", odom_y);
      ROS_INFO("w %f", odom_w*57.32);

      double angle_to_point = atan2((goal_y-odom_y),(goal_x-odom_x));
      double distance_goal = sqrt(pow((goal_x-odom_x),2)+pow((goal_y-odom_y),2));
      double diff = angle_to_point-odom_w;

      if (diff > M_PI)
        diff -= 2*M_PI;
      else if(diff < - M_PI)
        diff += 2*M_PI;

  double dist_point = sqrt(pow((goal_xarr.at(p+1)-odom_x),2)+pow((goal_yarr.at(p+1)-odom_y),2));

    if(goal_xarr.size() == 2 && dist_point < 0.5){
      near_end = true;
    }
    else if(dist_point < horizon && goal_xarr.size() > 2){
       goal_xarr.erase(goal_xarr.begin());
       goal_yarr.erase(goal_yarr.begin());
      }

      if(distance_goal < 0.1){
        ROS_INFO("final destination %f", distance_goal);
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;

        forward = false;

        as->setSucceeded();
      }
      else if(std::abs(diff) > 0.2){
        ROS_INFO("spinning");
        ROS_INFO("a2p %f", angle_to_point*57.32);
        ROS_INFO("diff %f", (diff)*57.32);
        ROS_INFO("abs %f ", std::abs(diff));
        ROS_INFO("distance %f \n", distance_goal);


        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0.75*sign(diff);

        forward = false;
      }
      else{
        ROS_INFO("Driving");
        ROS_INFO("a2p %f", angle_to_point*57.32);
        ROS_INFO("distance %f ", distance_goal);
        ROS_INFO("abs %f ", std::abs(diff));
        ROS_INFO("diff %f \n", (diff)*57.32);

        twist_msg.linear.x = 0.2;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 1*(diff);

        forward = true;
      }
    }

    //publish the twist
    twist_pub.publish(twist_msg);

    ros::spinOnce();

    loop_rate.sleep();}
}


int main(int argc, char **argv){
  ros::init(argc, argv, "look_forward_drive");
  ros::NodeHandle nh;
  Server server(nh, "path", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
