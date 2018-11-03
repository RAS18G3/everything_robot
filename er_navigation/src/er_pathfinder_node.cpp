#ifndef MAX_ITERS
#define MAX_ITERS 100
#endif

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "rrt_utils.h"

ros::Duration timeout(1.0);

ros::NodeHandle* npointer;

ros::Publisher path_pub;
ros::Subscriber path_sub;

void path_callback(const nav_msgs::Path::ConstPtr path)
{
  // get new map
  nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/occupancy_grid", *npointer, timeout));
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> map = occupancy_grid.data;
  //map = diluteMap(map, height, width, 10);
  double xStart = path->poses[0].pose.position.x;
  double yStart = path->poses[0].pose.position.y;
  double xGoal = path->poses[1].pose.position.x;
  double yGoal = path->poses[1].pose.position.y;

  nav_msgs::Path foundPath;

  RRTree tree;
  TreeNode root = {xStart, yStart, 0, NULL};
  tree.addNode(&root);
  for(int i = 0; i<MAX_ITERS; i++)
  {
    if(generateNode(&tree, map, width, height, xGoal, yGoal))
    {
      ROS_INFO("got to publisher!");
      foundPath = unpackPath(&tree);
      path_pub.publish(foundPath);
      return;
    }
  }
  ROS_INFO("No path found in %d iters", MAX_ITERS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::NodeHandle n;
  npointer = &n;
  path_sub = n.subscribe("/path1", 1, path_callback);
  path_pub = n.advertise<nav_msgs::Path>("/path2", 1);
  ros::spin();
  return 0;
}
