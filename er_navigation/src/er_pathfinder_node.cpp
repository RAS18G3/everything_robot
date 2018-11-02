#include "rrt_utils.h"

#include "ros/ros.h"
#include "ros/topic.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

// rrt will only go on for this many iters
int max_iters = 100;

ros::NodeHandle n;

void path_callback(const nav_msgs::Path path)
{
  nav_msgs::OccupancyGrid occupancy_grid;
  // get new map
  //nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/occupancy_grid", n, 1));
  int height = occupancy_grid.info.height;
  int width = occupancy_grid.info.width;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> map = occupancy_grid.data;
  //map = diluteMap(map, height, width, 10);

  double xStart = path.poses[0].pose.position.x;
  double yStart = path.poses[0].pose.position.y;
  double xGoal = path.poses[1].pose.position.x;
  double yGoal = path.poses[1].pose.position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pathfinder_node");
  ros::Subscriber path_sub = n.subscribe("path", 10, path_callback);
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/path", 1);
  ros::spin();
  return 0;
}
