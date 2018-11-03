#ifndef RRT_UTILS_H
#define RRT_UTILS_H

#include "ros/ros.h"
#include "occupancy_grid_utils.h"
#include "nav_msgs/Path.h"

double point_dist(double x1, double x2, double y1, double y2); // dist btw 2 pts
bool point_coll(double x1, double x2, double y1, double y2, std::vector<int8_t> map, int width, int height); // collision btw 2 pts

struct TreeNode
{
  double x, y;
  int depth;
  TreeNode* parent;

  void declareNode(double xx, double yy, TreeNode* parent); // this should probably be a proper constructor
};

struct RRTree
{
  int size; // total number of nodes
  std::vector<TreeNode*> nodes; // array of all the tree nodes

  void addNode(TreeNode* newNode);
};

int generateNode(RRTree* tree, std::vector<int8_t> map, int width, int height, double x_goal, double y_goal);

nav_msgs::Path unpackPath(RRTree* tree);

#endif
