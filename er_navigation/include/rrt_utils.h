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

double point_dist(double x1, double x2, double y1, double y2); // dist btw 2 pts
bool point_coll(double x1, double x2, double y1, double y2, std::vector<int8_t> map, int width, int height); // collision btw 2 pts

struct TreeNode
{
  double x, y;
  std::shared_ptr<TreeNode> parent;
  int depth;
};

struct RRTree
{
  int size; // total number of nodes
  std::vector<std::shared_ptr<TreeNode>> nodes; // array of all the tree nodes
  void addNode(TreeNode newNode);
};

void generateNode(RRTree* tree, std::vector<int8_t> map, int width, int height)


#endif
