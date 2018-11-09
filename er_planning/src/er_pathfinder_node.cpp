#ifndef MAX_TIME
#define MAX_TIME 15
#endif
#ifndef DILUTE_THRESH
#define DILUTE_THRESH 0.1
#endif
#ifndef COLL_THRESH
#define COLL_THRESH 50
#endif
#ifndef DEBUG
#define DEBUG 1
#endif

//#include "ros/ros.h"
#include "rrt_utils.h"
#include <actionlib/client/simple_action_client.h>
#include "nav_msgs/GetPlan.h"

// for the waitForMessage call
ros::NodeHandle* npointer;
ros::Duration timeout(3.0);
double time_start, time_now;

ros::Publisher pathfinderMap_pub;
ros::Publisher pathfinderPath_pub;
ros::ServiceServer pathfinder_srv;

bool path_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
  time_start = ros::Time::now().toSec();
  time_now = ros::Time::now().toSec();
  // get latest map
  nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/slam/occupancy_grid", *npointer, timeout));

  // dilute map
  occupancy_grid = diluteMap(occupancy_grid, DILUTE_THRESH);

  pathfinderMap_pub.publish(occupancy_grid);

  // unpack map info
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  double offsetX = occupancy_grid.info.origin.position.x;
  double offsetY = occupancy_grid.info.origin.position.y;
  std::vector<int8_t> map = occupancy_grid.data;

  // save start and goal in local positions (i.e. array indeces)
  double xStart = (req.start.pose.position.x-offsetX)/resolution;
  double yStart = (req.start.pose.position.y-offsetY)/resolution;
  double xGoal = (req.goal.pose.position.x-offsetX)/resolution;
  double yGoal = (req.goal.pose.position.y-offsetY)/resolution;

  bool invalidPath = false;
  if(occupancy_grid.data[(int)xStart + ((int)yStart)*width] > COLL_THRESH){ invalidPath = true; }
  if(occupancy_grid.data[(int)xGoal + ((int)yGoal)*width] > COLL_THRESH){ invalidPath = true; }
  if( invalidPath )
  {
    if( DEBUG==1 ){ ROS_INFO("Invalid Start or Goal."); }
    return false;
  }

  // init tree and root
  RRTree tree;
  TreeNode root = {xStart, yStart, 0, 0};
  tree.addNode(root);

  while(time_now-time_start < MAX_TIME)
  {
    TreeNode newNode = generateNode(tree, map, width, height, COLL_THRESH);
    if( newNode.x == 0 )
    // timed out
    {
      break;
    }
    tree.addNode(newNode);

    if(!point_coll(newNode.x, newNode.y, xGoal, yGoal, map, width, height, COLL_THRESH))
    {
      TreeNode lastNode = {xGoal, yGoal, newNode.depth+1, tree.size-1};
      tree.addNode(lastNode);
      nav_msgs::Path foundPath;
      foundPath.header = occupancy_grid.header;
      foundPath.poses = unpackPath(tree);
      foundPath.poses = smoothPath(foundPath.poses, occupancy_grid, COLL_THRESH);
      foundPath.poses = scalePath(foundPath.poses, offsetX, offsetY, resolution);
      res.plan = foundPath;
      if(DEBUG==1)
      {
        time_now = ros::Time::now().toSec();
        pathfinderMap_pub.publish(occupancy_grid);
        pathfinderPath_pub.publish(foundPath);
        ROS_INFO("Path found and published in %lf s.", time_now-time_start);
      }
      return true;
    }
    time_now = ros::Time::now().toSec();
  }
  if( DEBUG==1 ){ ROS_INFO("No path found."); }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::NodeHandle n;
  npointer = &n;
  ros::Rate loop_rate(10);

  pathfinder_srv = n.advertiseService("/pathfinder/find_path", &path_callback);

  // only for Rviz debug
  pathfinderMap_pub = n.advertise<nav_msgs::OccupancyGrid>("/pathfinder_map",1);
  pathfinderPath_pub = n.advertise<nav_msgs::Path>("/pathfinder_path",1);

  while(ros::ok)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
