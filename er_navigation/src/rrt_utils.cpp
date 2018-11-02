#ifndef THRESHOLD
#define THRESHOLD 50
#endif

#include "rrt_utils.h"

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

bool point_coll(double x1, double y1, double x2, double y2, std::vector<int8_t> map, int height, int width)
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
    if (map[x_floor + y_floor*width] > THRESHOLD)
    {
      return true;
    }
  }
  return false; // fall through case
}

RRTree::RRTree()
// main RRT object
{
  int size = 0;
  std::vector<std::shared_ptr<TreeNode>> nodes;

  void addNode(std::shared_ptr<TreeNode> newNode)
  {
    nodes.push_back(newNode);
    size++;
  }
};

TreeNode::TreeNode(double xx, double yy, TreeNode parentNode)
{
  double x = xx;
  double y = yy;
  int depth = 0;
  TreeNode parent = NULL;
  if(parentNode != NULL)
  {
    depth = parentNode.depth + 1;
    parent = parentNode;
  }
};

int generateNode(RRTree* tree, std::vector<int8_t> map, int width, int height, double xGoal, double yGoal)
// generate RRT node randomly
// returns 0 if node has clear path to goal
{
  int tree_size = tree.size;

  while(1)
  {
    // generate random x and y
    int x = rand() % width;
    int y = rand() % height;

    // calculate distances to all existing nodes
    std::vector<double> dists(tree_size,0);
    for(int i = 0; i<tree_size; i++)
    {
      x_node = tree.nodes[i].x;
      y_node = tree.nodes[i].y;
      dists[i] = point_dist(x,y,x_node,y_node);
    }

    // find closest node and check if there is a path
    while( !dists.empty() )
      std::vector<int>::iterator closest = std::min_element(std::begin(dists), std::end(dists));
      int closest_index = std::distance(std::begin(dists), closest);
      x_node = tree.nodes[closest_index].x;
      y_node = tree.nodes[closest_index].y;

      if ( !point_coll(x, y, x_node, y_node, map, width, height) )
      // found valid node
      {
        newNode = TreeNode(x_node, y_node, &tree.nodes[closest_index]);
        tree.addNode(newNode);
        return point_coll(x_node, y_node, x_goal, y_goal, map, width, height);
      }

      // fall through, erase closest point from list if invalid
      dists.erase(std::begin(dists)+closest_index);
  }
}
