#include "ros.h"
#include "nav_msgs/Odometry"
#include <actionlib/client/simple_action_client.h>


int main (int argc, char **argv){

  actionlib::SimpleActionClient<nav_msgs::Odometry> ac("goal", true);

  ac.waitForServer();

  nav_msgs::Odometry goal;
  goal.pose.pose.position.x = 1;
  goal.pose.pose.position.y = 1;
  ac.sendGoal(goal);

  bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));


     if (finished_before_timeout)
     {
       actionlib::SimpleClientGoalState state = ac.getState();
       ROS_INFO("Action finished: %s",state.toString().c_str());
     }
     else
       ROS_INFO("Action did not finish before the time out.");

     //exit
    return 0;

}
