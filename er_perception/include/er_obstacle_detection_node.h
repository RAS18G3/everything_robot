#ifndef ER_OBSTACLE_DETECTION_NODE_H
#define ER_OBSTACLE_DETECTION_NODE_H

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "std_msgs/Bool.h"

#include <string>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class ObstacleDetectionNode {
public:
  ObstacleDetectionNode();
  ~ObstacleDetectionNode();

  void run_node();

private:
  void init_node();
  void pointcloud_cb(const PointCloud::ConstPtr& msg);

  // the range in which points will be counted
  double range_;
  // the number of points inside this radius, s.t. obstacle will be published
  int threshold_;

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_subsriber_;
  ros::Publisher obstacle_publisher_;
};


#endif
