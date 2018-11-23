#ifndef RRT_UTILS_H
#define RRT_UTILS_H

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float64MultiArray.h"

struct TreeNode
{
  double x, y;
  int depth;
  int parent_index;
};

struct RRTree
{
  int size = 0; // total number of nodes
  std::vector<TreeNode> nodes; // array of all the tree nodes

  void addNode(TreeNode newNode);
};

double point_dist(double x1, double x2, double y1, double y2); // dist btw 2 pts

bool point_coll(double x1, double x2, double y1, double y2, std::vector<int8_t> map, int width, int height, int collThresh); // collision btw 2 pts

TreeNode generateNode(RRTree tree, std::vector<int8_t> map, int width, int height, int collThresh);

RRTree intermediateNodes(RRTree tree, TreeNode newNode, double step_thresh);

nav_msgs::OccupancyGrid diluteMap(nav_msgs::OccupancyGrid occupancy_grid, double diluteThresh); // for map dilution

std::vector<geometry_msgs::PoseStamped> unpackPath(RRTree tree);

std::vector<geometry_msgs::PoseStamped> smoothPath(std::vector<geometry_msgs::PoseStamped> path, nav_msgs::OccupancyGrid map, int collThresh);

std::vector<geometry_msgs::PoseStamped> scalePath(std::vector<geometry_msgs::PoseStamped> path, double offsetX, double offsetY, double resolution);

#endif
