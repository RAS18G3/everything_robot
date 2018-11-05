#ifndef MAX_NODES
#define MAX_NODES 1000
#endif
#ifndef DILUTE_THRESH
#define DILUTE_THRESH 0.2
#endif
#ifndef DEBUG
#define DEBUG 1
#endif

//#include "ros/ros.h"
#include "rrt_utils.h"
#include <actionlib/client/simple_action_client.h>

// for the waitForMessage call
ros::NodeHandle* npointer;
ros::Duration timeout(1.0);

// for action client
typedef actionlib::SimpleActionClient<er_navigation::Pathfinder> Client;
//ros::Publisher path_pub;
ros::Publisher dilMap_pub;
//ros::Subscriber path_sub;
//ros::Subscriber map_sub;

/*
nav_msgs::OccupancyGrid occupancy_grid;
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr received_grid)
{
  // update map
  occupancy_grid.header = received_grid->header;
  occupancy_grid.info = received_grid->info;
  occupancy_grid.data = received_grid->data;
}
*/
void path_callback(const nav_msgs::Path::ConstPtr path)
{
  if( DEBUG == 1){ double exec_time = ros::Time::now().toSec(); }
  // get latest map
  nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/occupancy_grid", *npointer, timeout));
  // dilute map and publish for debug
  occupancy_grid = diluteMap(occupancy_grid, DILUTE_THRESH);
  dilMap_pub.publish(occupancy_grid);

  // unpack map info
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  double offsetX = occupancy_grid.info.origin.position.x;
  double offsetY = occupancy_grid.info.origin.position.y;
  std::vector<int8_t> map = occupancy_grid.data;

  // save start and goal in local positions (i.e. array indeces)
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
    if( newNode.x == 0 )
    // timed out
    {
      //what do?
    }
    tree.addNode(newNode);

    if(!point_coll(newNode.x, newNode.y, xGoal, yGoal, map, width, height))
    {
      TreeNode lastNode = {xGoal, yGoal, newNode.depth+1, tree.size-1};
      tree.addNode(lastNode);
      break;
    }
  }
  //nav_msgs::Path foundPath = nav_msgs::Path();
  //foundPath.poses = unpackPath(tree, resolution, offsetX, offsetY);

  //foundPath.header = occupancy_grid.header;
  //path_pub.publish(foundPath);
  std_msgs::Float64MultiArray path;
  path = unpackPath2(tree, resolution, offsetX, offsetY);

  // action client ?
  client.sendGoal(path);
  // debug only

  if(DEBUG==1)
  {
    exec_time -= t_start.toSec();
    ROS_INFO("found and published in %lf s", exec_time);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::NodeHandle n;
  npointer = &n;
  ros::Rate loop_rate(10);

  //path_pub = n.advertise<nav_msgs::Path>("/path2", 1);
  dilMap_pub = n.advertise<nav_msgs::OccupancyGrid>("/dilutedMap",1);
  //path_sub = n.subscribe("/path1", 1, path_callback);
  //map_sub = n.subscribe("/occupancy_grid", 1, map_callback);

  while(ros::ok)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
