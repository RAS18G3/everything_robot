#ifndef THRESHOLD
#define THRESHOLD 50
#endif

#include "rrt_utils.h"

double point_dist(double x1, double y1, double x2, double y2)
// gives squared distance between points p1 and p2
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

  while(1)
  {
    // step once along vector
    x1 += vec_x;
    y1 += vec_y;

    if (point_dist(x1, y1, x2, y2) < 1)
    // short distance . we are at the goal
    {
      break;

    }
    // basic rounding of indeces (recasting.floor)
    int x_floor = (int) x1;
    int y_floor = (int) y1;
    if (map[x_floor + y_floor*width] > THRESHOLD)
    {
      return true;
    }
  }
  return false; // fall through case
}

void RRTree::addNode(TreeNode newNode)
// adds node to the tree
{
  nodes.push_back(newNode);
  size++;
}

TreeNode generateNode(RRTree tree, std::vector<int8_t> map, int width, int height)
// generate RRT node randomly, return nullptr if unsuccesful
{
  int tree_size = tree.size;
  while(1)
  {
    // generate random x and y
    double x = rand() % width+1;
    double y = rand() % height+1;
    double x_node, y_node;

    // calculate distances to all existing nodes
    std::vector<double> dists(tree_size,0);
    for(int i = 0; i<tree_size; i++)
    {
      x_node = tree.nodes[i].x;
      y_node = tree.nodes[i].y;
      dists[i] = point_dist(x,y,x_node,y_node);
    }

    // find closest node and check if there is a path
    while(!dists.empty())
    {
      std::vector<double>::iterator closest = std::min_element(std::begin(dists), std::end(dists));
      int closest_index = std::distance(std::begin(dists), closest);

      x_node = tree.nodes[closest_index].x;
      y_node = tree.nodes[closest_index].y;

      if ( !point_coll(x, y, x_node, y_node, map, width, height) )
      // this node is valid!
      {
        int depth = tree.nodes[closest_index].depth+1;
        TreeNode newNode = {x, y, depth, closest_index};

        return newNode;
        break;
      }

      // fall through, erase closest point from list if invalid
      dists.erase(std::begin(dists)+closest_index);
    }
  }
}

nav_msgs::Path unpackPath(RRTree tree)
// assumes last node points is at goal and unpacks that path
{
  int tree_size = tree.size;
  TreeNode currentNode = tree.nodes[tree_size-1];
  int depth = currentNode.depth;

  nav_msgs::Path path;
  std_msgs::Header path_header;

  path_header.frame_id = "global";
  std::vector<geometry_msgs::PoseStamped> poses(depth);

  while(1)
  {
    poses[depth].pose.position.x = currentNode.x;
    poses[depth].pose.position.y = currentNode.y;
    depth = currentNode.depth;
    ROS_INFO("path step %d: %lf %lf",depth,currentNode.x,currentNode.y);

    if(depth==0){break;} // break if root

    // dive in
    currentNode = tree.nodes[currentNode.parent_index];
    depth = currentNode.depth;
  }
  path.header = path_header;
  path.poses = poses;
  return path;
}
