#ifndef ER_SLAM_NODE_H
#define ER_SLAM_NODE_H

#include "map_reader.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"

#include <string>

class SLAMNode {
public:
  SLAMNode();
  ~SLAMNode();

  void run_node();

private:
};


#endif
