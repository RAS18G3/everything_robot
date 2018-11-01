#ifndef THRESHOLD
#define THRESHOLD 50
#endif

#include "rrt_utils.h"
#include "occupancy_grid_utils.h"

bool checkMap( double x, double y, nav_msgs::OccupancyGrid &occupancy_grid)
// returns "occupancy" of the grid, i.e. if prblty of obstacle is >THRESHOLD
{
  // basic rounding of indeces (recasting->floor)
  int x_floor = (int) x;
  int y_floor = (int) y;
  return (at(occupancy_grid, x_floor, y_floor) > THRESHOLD);
}

double point_dist(double x1, double y1, double x2, double y2)
// gives estimate of distance between points p1 and p2
// not true distance but good enough for comparisons
{
  // reusing vars -- with so many calls we might as well be cheap
  x1 -= x2;
  y1 -= y2;

  // multiplication instead of power is apparently ever so slightly faster
  x1 *= x1;
  y1 *= y1;
  return (x1+y1);
}

bool point_coll(double x1, double y1, double x2, double y2, nav_msgs::OccupancyGrid &occupancy_grid)
// checks for collision between points by stepping along a vector
{
  int x,y; // for indeces
  double vec_x = x2-x1; // dx
  double vec_y = y2-y1; // dy

  // normalize vector
  double vec_magnitude = sqrt(vec_x*vec_x + vec_y*vec_y);
  vec_x /= vec_magnitude;
  vec_y /= vec_magnitude;

  while 1
  {
    // step once along vector
    x1 += vec_x;
    y1 += vec_y;

    if (dist(x1, y1, x2, y2) < 1)
    // short distance -> we are at the goal
    {
      break;

    }
    // basic rounding of indeces (recasting->floor)
    int x_floor = (int) x1;
    int y_floor = (int) y1;
    if (at(occupancy_grid, x_floor, y_floor) > THRESHOLD)
    {
      return true;
    }
  }
  return false; // fall through case
}

RRTree::RRTree()
{
  int size = 0;
  std::vector<std::shared_ptr<TreeNode>> nodes;

  void addNode(double x, double y, std::shared_ptr<TreeNode> parent)
  {

    nodes.push_back(newNode);
    size++;
  }
}

TreeNode::TreeNode(double xx, double yy, std::shared_ptr<TreeNode> parentNode)
{
  double x = xx;
  double y = yy;
  int depth = 0;
  std::shared_ptr<TreeNode> parent = NULL;
  if(parentNode != NULL)
  {
    depth = parentNode.depth + 1;
    parent = parentNode;
  }
}
