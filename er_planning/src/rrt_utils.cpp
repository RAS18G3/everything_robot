#include "rrt_utils.h"

void RRTree::addNode(TreeNode newNode)
// adds node to the tree
{
  nodes.push_back(newNode);
  size++;
}

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

bool point_coll(double x1, double y1, double x2, double y2, std::vector<int8_t> map, int height, int width, int collThresh)
// checks for collision between points by stepping along a vector
{
  int x,y; // for indeces
  double vec_x = x2-x1; // dx
  double vec_y = y2-y1; // dy

  // normalize vector
  double vec_magnitude = 2*sqrt(vec_x*vec_x + vec_y*vec_y);
  vec_x /= vec_magnitude;
  vec_y /= vec_magnitude;

  while(1)
  {
    // step once along vector
    x1 += vec_x;
    y1 += vec_y;
    double d = point_dist(x1,y1,x2,y2);
    //if (point_dist(x1, y1, x2, y2) < 1)
    if(d<1)
    // short distance . we are at the goal
    {
      break;
    }
    // basic rounding of indeces (recasting.floor)
    int x_floor = (int) x1;
    int y_floor = (int) y1;
    if (map[x_floor + y_floor*width] > collThresh)
    {
      return true;
    }
  }
  return false; // fall through case
}

TreeNode generateNode(RRTree tree, std::vector<int8_t> map, int width, int height, int collThresh)
// generate RRT node randomly, return nullptr if unsuccesful
{
  int tree_size = tree.size;
  while(1)
  {
    // generate random x and y
    double x = rand() % width+1;
    double y = rand() % height+1;

    double x_node, y_node, dist;
    bool node_invalid = false;

    // check if new node is on occupied point, throw away if it is
    int x_floor = (int) x;
    int y_floor = (int) y;
    if (map[x_floor + y_floor*width] > collThresh){ continue; }
    // calculate distances to all existing nodes
    std::vector<double> dists(tree_size,0);
    for(int i = 0; i<tree_size; i++)
    {
      x_node = tree.nodes[i].x;
      y_node = tree.nodes[i].y;
      dist = point_dist(x,y,x_node,y_node);
      if (dist==0) // node already exists, throw away
      {
        node_invalid = true;
        break;
      }
      dists[i] = dist;
    }

    // find closest node and check if there is a path
    if( node_invalid ){ continue; }
    while(!dists.empty())
    {
      std::vector<double>::iterator closest = std::min_element(std::begin(dists), std::end(dists));
      int closest_index = std::distance(std::begin(dists), closest);

      x_node = tree.nodes[closest_index].x;
      y_node = tree.nodes[closest_index].y;
      if ( !point_coll(x, y, x_node, y_node, map, width, height, collThresh) )
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

nav_msgs::OccupancyGrid diluteMap(nav_msgs::OccupancyGrid occupancy_grid, double diluteThresh)
// SUPER-lazy map dilution
{
  // lazy copy of original map
  nav_msgs::OccupancyGrid dilutedMap = occupancy_grid;

  // unpack
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> mapOrig = occupancy_grid.data;
  std::vector<int8_t> mapNew = occupancy_grid.data;

  int dilRange = (int) (diluteThresh/resolution); // length of blob side

  int w1, w2, wmin, wmax, h1, h2, hmin, hmax;//

  for(w1 = 0; w1 < width; w1++)
  {
    wmin = std::max(w1-dilRange,0);
    wmax = std::min(w1+dilRange,width-1);
    for(h1 = 0; h1 < height; h1++)
    {
      hmin = std::max(h1-dilRange,0);
      hmax = std::min(h1+dilRange,height-1);
      for(w2 = wmin; w2 < wmax; w2++)
      {
        for(h2 = hmin; h2 < hmax; h2++)
        {
          mapNew[w2 + h2*width] = std::max(mapOrig[w1 + h1*width], mapNew[w2 + h2*width]);
        }
      }
    }
  }
  dilutedMap.data = mapNew;
  return dilutedMap;
}

std::vector<geometry_msgs::PoseStamped> unpackPath(RRTree tree)
// assumes last node points is at goal and unpacks that path
{
  int tree_size = tree.size;
  TreeNode currentNode = tree.nodes[tree_size-1];
  int depth = currentNode.depth;

  std::vector<geometry_msgs::PoseStamped> poses(depth+1);

  while(1)
  {
    poses[depth].pose.position.x = currentNode.x;
    poses[depth].pose.position.y = currentNode.y;

    depth = currentNode.depth;
    if(depth==0){break;} // break if root

    // dive in
    int parent_index = currentNode.parent_index;
    currentNode = tree.nodes[currentNode.parent_index];
    depth = currentNode.depth;
  }
  return poses;
}

std::vector<geometry_msgs::PoseStamped> smoothPath(std::vector<geometry_msgs::PoseStamped> path, nav_msgs::OccupancyGrid map, int collThresh)
// simple path smoother
{
  double x1, x2, y1, y2;
  std::vector<geometry_msgs::PoseStamped> newPath = path;
  int size = newPath.size();
  bool changed = true;

  while( changed && size>2)
  {
    size = newPath.size();
    changed = false;
    for(int i=0; i < size-2; i++)
    {
      x1 = newPath[i].pose.position.x;
      y1 = newPath[i].pose.position.y;
      x2 = newPath[i+2].pose.position.x;
      y2 = newPath[i+2].pose.position.y;
      if( !point_coll(x1, y1, x2, y2, map.data, map.info.height, map.info.width, collThresh) )
      {
        newPath.erase(std::begin(newPath)+i+1);
        changed = true;
        break;
      }
    }
  }
  return newPath;
}

std::vector<geometry_msgs::PoseStamped> scalePath(std::vector<geometry_msgs::PoseStamped> path, double offsetX, double offsetY, double resolution)
{
  std::vector<geometry_msgs::PoseStamped> newPath = path;
  int size = newPath.size();
  for(int i=0; i<size; i++)
  {
    newPath[i].pose.position.x = path[i].pose.position.x*resolution+offsetX;
    newPath[i].pose.position.y = path[i].pose.position.y*resolution+offsetY;
  }
  return newPath;
}
