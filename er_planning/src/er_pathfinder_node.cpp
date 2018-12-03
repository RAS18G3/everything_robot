#ifndef MAX_TIME
#define MAX_TIME 5
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
//ros::NodeHandle* npointer;
ros::Duration timeout(3.0);
double time_start, time_now;

double dilute_threshold;

ros::Publisher pathfinderMap_pub;
ros::Publisher pathfinderPath_pub;
ros::Subscriber occupancy_grid_sub;
ros::ServiceServer pathfinder_srv;

nav_msgs::OccupancyGrid occupancy_grid;

void map_update_callback(const nav_msgs::OccupancyGrid::ConstPtr& occupancy_grid_msg)
{
  occupancy_grid = *occupancy_grid_msg;
}

bool path_callback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
{
  time_start = ros::Time::now().toSec();
  time_now = ros::Time::now().toSec();

  // get latest map
  //nav_msgs::OccupancyGrid occupancy_grid = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/slam/occupancy_grid", *npointer, timeout));

  // dilute map
  nav_msgs::OccupancyGrid undiluted_grid = occupancy_grid;
  occupancy_grid = diluteMap(occupancy_grid, dilute_threshold);


  // unpack map info
  int width = occupancy_grid.info.width;
  int height = occupancy_grid.info.height;
  double resolution = occupancy_grid.info.resolution;
  double offsetX = occupancy_grid.info.origin.position.x;
  double offsetY = occupancy_grid.info.origin.position.y;
  std::vector<int8_t>& map = occupancy_grid.data;

  // save start and goal in local positions (i.e. array indeces)
  double xStart = (req.start.pose.position.x-offsetX)/resolution;
  double yStart = (req.start.pose.position.y-offsetY)/resolution;
  double xGoal = (req.goal.pose.position.x-offsetX)/resolution;
  double yGoal = (req.goal.pose.position.y-offsetY)/resolution;

  bool invalidPath = false;

  if(occupancy_grid.data[(int)xStart + ((int)yStart)*width] > COLL_THRESH) {
    // remove dilution around start point (if robot drives into some area it should be able to get out of it)
    ROS_INFO("Start in diluted area, remove dilution around it");
    const int remove_radius = dilute_threshold*1 / resolution;
    for(int x=xStart - remove_radius; x < xStart + remove_radius; ++x) {
      for(int y=yStart - remove_radius; y < yStart + remove_radius; ++y) {
        if(undiluted_grid.data[x + y*width ] <= 50) {
          map[x + y*width] = 0;
        }
      }
    }
  }

  // remove dilution around end point (if object is close to a wall, path execution will stop anyways if not possible)
  if(occupancy_grid.data[(int)xGoal + ((int)yGoal)*width] > COLL_THRESH) {
    ROS_INFO("Goal in diluted area, remove dilution around it");
    const int small_remove_radius = 0.055 / resolution;
    // remove mapped stuff as well (since objects are obstacles themselves, thus they will always lie on occupied fields)
    for(int x=xGoal - small_remove_radius; x < xGoal + small_remove_radius; ++x) {
      for(int y=yGoal - small_remove_radius; y < yGoal + small_remove_radius; ++y) {
        if(undiluted_grid.data[x + y*width ] <= 99) { // do not remove actual walls
          map[x + y*width] = 0;
        }
      }
    }
    // remove dilution with larger radius
    const int remove_radius = dilute_threshold*1.1 / resolution;
    for(int x=xGoal - remove_radius; x < xGoal + remove_radius; ++x) {
      for(int y=yGoal - remove_radius; y < yGoal + remove_radius; ++y) {
        if(undiluted_grid.data[x + y*width ] <= 50) {
          map[x + y*width] = 0;
        }
      }
    }
  }

  pathfinderMap_pub.publish(occupancy_grid);

  /* not relevant anymore due to un-dilutions above
  if(undiluted_grid.data[(int)xGoal + ((int)yGoal)*width] >= 100){ invalidPath = true; }
  if(undiluted_grid.data[(int)xStart + ((int)yStart)*width] >= 100){ invalidPath = true; }
  if( invalidPath )
  {
    if( DEBUG==1 ){ ROS_INFO("Invalid Start or Goal."); }
    return false;
  }
  */

  // init tree and root
  RRTree tree;
  TreeNode root = {xStart, yStart, 0, 0};
  tree.addNode(root);

  while(time_now-time_start < MAX_TIME)
  {
    // check if latest tree node has line of sight to the goal
    TreeNode newNode = tree.nodes[tree.size-1];
    if(!point_coll(newNode.x, newNode.y, xGoal, yGoal, map, width, height, COLL_THRESH))
    {
      // finalize if it has - add goal node, translate tree to path etc
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

    // fall-through else:
    newNode = generateNode(tree, map, width, height, COLL_THRESH);
    if( newNode.x == 0 )
    // timed out
    {
      break;
    }
    tree.addNode(newNode);

    time_now = ros::Time::now().toSec();
  }

  // if the while-loop times out:
  if( DEBUG==1 ){ ROS_INFO("No path found in given time."); }
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_pathfinder_node");
  ros::NodeHandle n;
  //npointer = &n;

  pathfinder_srv = n.advertiseService("/pathfinder/find_path", &path_callback);

  occupancy_grid_sub = n.subscribe("/slam/occupancy_grid",1,map_update_callback);

  // load parameter
  ros::param::param<double>("~dilusion_radius", dilute_threshold, 0.2);

  // only for Rviz debug
  pathfinderMap_pub = n.advertise<nav_msgs::OccupancyGrid>("/pathfinder_map",1);
  pathfinderPath_pub = n.advertise<nav_msgs::Path>("/pathfinder_path",1);

  ros::spin();
  ros::Rate loop_rate(10);
/*  while(ros::ok)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }*/
  return 0;
}
