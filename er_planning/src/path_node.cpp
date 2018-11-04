#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <er_planning/PathAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<er_planning::PathAction> Client;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "path_node");
  Client client("path", true);
  client.waitForServer();
  er_planning::PathGoal goal_path;
  std::vector<float> x = {0, 0, 2, 0, 2, 2, 2, 3};
  goal_path.Path = x;
  client.sendGoal(goal_path);
  client.waitForResult(ros::Duration(5.0));
  return 0;
}
