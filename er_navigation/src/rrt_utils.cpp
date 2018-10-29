#ifndef THRESHOLD
#define THRESHOLD 50
#endif

#include "rrt_utils.h"
#include "occupancy_grid_utils.h"

int threshold = THRESHOLD;

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

    // "round" (hm, floor. good enough)
    x = (int) x1;
    y = (int) y1;
    if (at(occupancy_grid, x, y) > threshold)
    {
      return true;
    }
  }
  return false; // fall through case
}

//RRTree::RRTree

/*
  rrt class sketch
  class Tree
    int .size = number of nodes
    Node[] .nodes = array of node pointers
                    has to be accessible through index
                    for easy iteration through all nodes
  class Node
    double .posX
    double .posY
    Node .parent = pointer to parent node


  idea - could make
*/
