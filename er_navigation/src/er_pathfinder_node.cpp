#ifndef MAX_NODES
#define MAX_NODES 1000
#endif
#ifndef DILUTE_THRESH
#define DILUTE_THRESH 0.1
#endif

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "rrt_utils.h"

ros::Publisher path_pub;
ros::Publisher dilMap_pub;
ros::Subscriber path_sub;
ros::Subscriber map_sub;

nav_msgs::OccupancyGrid occupancy_grid;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr received_grid)
{
  // update map
  occupancy_grid.header = received_grid->header;
  occupancy_grid.info = received_grid->info;
  occupancy_grid.data = received_grid->data;
}

void path_callback(const nav_msgs::Path::ConstPtr path)
{
  ros::Time t_start = ros::Time::now();
  // dilute map
  occupancy_grid = diluteMap(occupancy_grid, DILUTE_THRESH);
  dilMap_pub.publish(occupancy_grid);
  // unpack map
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  std::vector<int8_t> map = occupancy_grid.data;
  double offsetX = occupancy_grid.info.origin.position.x;
  double offsetY = occupancy_grid.info.origin.position.y;

  ROS_INFO("map info: w %d, h %d, r %lf",width,height,resolution);

  // dilute map
  //map = diluteMap(map, height, width, 10);

  // save start and goal in local positions
  double xStart = (path->poses[0].pose.position.x-offsetX)/resolution;
  double yStart = (path->poses[0].pose.position.y-offsetY)/resolution;
  double xGoal = (path->poses[1].pose.position.x-offsetX)/resolution;
  double yGoal = (path->poses[1].pose.position.y-offsetY)/resolution;

  // init tree and root
  RRTree tree;
  TreeNode root = {xStart, yStart, 0, 0};
  tree.addNode(root);

  for(int i = 0; i<MAX_NODES; i++)
  {
    TreeNode newNode = generateNode(tree, map, width, height);
    tree.addNode(newNode);

    if(!point_coll(newNode.x, newNode.y, xGoal, yGoal, map, width, height))
    {
      TreeNode lastNode = {xGoal, yGoal, newNode.depth+1, tree.size-1};
      tree.addNode(lastNode);
      break;
    }
  }
  nav_msgs::Path foundPath = nav_msgs::Path();
  foundPath.poses = unpackPath(tree, resolution, offsetX, offsetY);
  foundPath.header = occupancy_grid.header;
  path_pub.publish(foundPath);

  // debug only

  ros::Time t_end = ros::Time::now();
  double secs = t_end.toSec() - t_start.toSec();
  ROS_INFO("found and published in %lf s", secs);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  path_pub = n.advertise<nav_msgs::Path>("/path2", 1);
  dilMap_pub = n.advertise<nav_msgs::OccupancyGrid>("/dilutedMap",1);
  path_sub = n.subscribe("/path1", 1, path_callback);
  map_sub = n.subscribe("/occupancy_grid", 1, map_callback);

  while(ros::ok)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
