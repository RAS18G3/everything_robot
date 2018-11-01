#ifndef RRT_UTILS_H
#define RRT_UTILS_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "occupancy_grid_utils.h"

/*
#include <cmath>
#include <string>
#include <fstream>
#include <sstream>
*/

double point_dist(); // dist btw 2 pts
bool point_coll(); // collision btw 2 pts


struct RRTree
{
  int size; // total number of nodes
  std::vector<std::shared_ptr<TreeNode>> nodes; // array of all the tree nodes
  void addNode();
};

struct TreeNode
{
  double x, y;
  std::shared_ptr<TreeNode> parent;
  int depth;
};

#endif
