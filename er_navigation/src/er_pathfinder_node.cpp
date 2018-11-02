#include "rrt_utils.h"

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

// rrt will only go on for this many iters
int max_iters = 100;

ros::NodeHandle n;
ros::Duration timeout(1.0);

void path_callback(const nav_msgs::Path::ConstPtr path)
{
  // get new map
  nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/occupancy_grid", n, timeout));
  int height = occupancy_grid.info.height;
  int width = occupancy_grid.info.width;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> map = occupancy_grid.data;
  //map = diluteMap(map, height, width, 10);

  double xStart = path.poses[0].pose.position.x;
  double yStart = path.poses[0].pose.position.y;
  double xGoal = path.poses[1].pose.position.x;
  double yGoal = path.poses[1].pose.position.y;

  RRTree tree;
  struct TreeNode root = {xStart, yStart, NULL};
  tree.addNode(root);

  for(int i = 0; i<max_iters; i++)
  {
    if(~generateNode(tree, map, width, height))
    {
      break;
      ROS_INFO("path found!!! But the scope runs out now :^)");
    }

  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::Subscriber path_sub = n.subscribe("/path1", 10, path_callback);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path2", 1);
  ros::spin();
  return 0;
}
