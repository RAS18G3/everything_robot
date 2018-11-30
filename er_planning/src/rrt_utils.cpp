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

bool point_coll(double x1, double y1, double x2, double y2, std::vector<int8_t> map, int width, int height, int collThresh)
// checks for collision between points by stepping along a vector
{
  int x,y; // for indeces
  double vec_x = x2-x1; // dx
  double vec_y = y2-y1; // dy

  // normalize vector
  double vec_magnitude = 2*sqrt(vec_x*vec_x + vec_y*vec_y);
  vec_x /= 2*vec_magnitude;
  vec_y /= 2*vec_magnitude;

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
  // timeout check
  double time_start, time_now;
  time_start = ros::Time::now().toSec();
  time_now = ros::Time::now().toSec();

  int tree_size = tree.size;
  while(time_now-time_start < 10)
  {
    time_now = ros::Time::now().toSec();
    // generate random x and y
    srand (time(NULL)); // get new random seed
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
        TreeNode newNode = {x, y, tree.nodes[closest_index].depth+1, closest_index};
        return newNode;
      }

      // fall through, erase closest point from list if invalid
      dists.erase(std::begin(dists)+closest_index);
    }
  }
  // fallthrough failed case
  TreeNode newNode = {-1,-1,-1,-1};
  return newNode;
}

RRTree intermediateNodes(RRTree tree, TreeNode newNode, double step_dist)
// creates intermediate nodes btw newNode and its parent, with interdistance step_dist
{
  int parent_index = newNode.parent_index;
  TreeNode parentNode = tree.nodes[parent_index];

  // x- and y-componentes of vec from newNode to parent
  double xvec = newNode.x - parentNode.x;
  double yvec = newNode.y - parentNode.y;

  // steps = total amount of new nodes, placed ca step_dist apart
  int steps = (int) (sqrt(xvec*xvec+yvec*yvec)/step_dist);

  // rescale vector for single steps
  xvec /= steps;
  yvec /= steps;

  for(int i=0; i<steps; i++)
  // creates intermediate nodes and pushes them to tree, including newNode
  {
    TreeNode interNode = {parentNode.x+xvec, parentNode.y+yvec, parentNode.depth+1, parent_index};
    tree.addNode(interNode);
    parent_index = tree.size-1;
    parentNode = tree.nodes[parent_index];
  }
  return tree;
}

nav_msgs::OccupancyGrid diluteMap(nav_msgs::OccupancyGrid occupancy_grid, double diluteThresh)
// _improved_ map dilution
{
  // copy of original map
  nav_msgs::OccupancyGrid dilutedMap = occupancy_grid;

  // unpack
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> mapOrig = occupancy_grid.data;
  std::vector<int8_t> mapNew = occupancy_grid.data;
  int dilRange = (int) (diluteThresh/resolution); // blob radius

  int x, y;
  int dist, circ_ind;
  int dilRangeSq = dilRange*dilRange;
  std::vector<int> circ_inds;

  // generate local indeces for a circle with dilRange radius
  for(x=-dilRange; x<dilRange; x++)
  {
    for(y=-dilRange; y<dilRange; y++)
    {
      dist = x*x + y*y; // no sqrt for quicker calc
      circ_ind = x+width*y;
      if(dist < dilRangeSq){ circ_inds.push_back(circ_ind); }
    }
  }

  int ind, map_ind;
  int ind_max = width*height;
  int circ_size = circ_inds.size();

  for(ind=0; ind<ind_max; ind++)
  // for each occupied point, create a circle of occupied points around it
  {
    if(mapOrig[ind]<50){ continue; } // skip if not occupied, hardcoded thresh
    for( circ_ind=0; circ_ind<circ_size; circ_ind++ )
    {
      map_ind = ind+circ_inds[circ_ind]; // "global" index
      if(map_ind > -1 && map_ind < ind_max )
      // out-of-bounds check
      {
        mapNew[map_ind] = 90;
      }
    }
  }
  dilutedMap.data = mapNew;
  return dilutedMap;
}

std::vector<geometry_msgs::PoseStamped> unpackPath(RRTree tree)
// assumes last node points at goal and unpacks that path
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
// improved path smoother, more aggresive but also heavier
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
      // now looks for the furthest point possible to cut out
      for(int j=size-1; j > i+1; j--)
      {
        x1 = newPath[i].pose.position.x;
        y1 = newPath[i].pose.position.y;
        x2 = newPath[j].pose.position.x;
        y2 = newPath[j].pose.position.y;
        if( !point_coll(x1, y1, x2, y2, map.data, map.info.width, map.info.height, collThresh) )
        // destroy all the points inbetween
        {
          for(int k=j-1; k>i; k--){ newPath.erase(std::begin(newPath)+k); }
          changed = true;
        }
        if(changed){ break; }
      }
      if(changed){ break; }
    }
  }
  return newPath;
}

/* OLD SMOOTHING
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
      if( !point_coll(x1, y1, x2, y2, map.data, map.info.width, map.info.height, collThresh) )
      {
        newPath.erase(std::begin(newPath)+i+1);
        changed = true;
        break;
      }
    }
  }
  return newPath;
}
*/

std::vector<geometry_msgs::PoseStamped> scalePath(std::vector<geometry_msgs::PoseStamped> path, double offsetX, double offsetY, double resolution)
// rescales path from grid indeces to global positions
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
