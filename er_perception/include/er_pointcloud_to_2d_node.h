#ifndef ER_POINTCLOUD_TO_2D_NODE_H
#define ER_POINTCLOUD_TO_2D_NODE_H

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "tf2_ros/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include <string>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class PointCloudTo2DNode {
public:
  PointCloudTo2DNode();
  ~PointCloudTo2DNode();

  void run_node();

private:
  void init_node();
  void pointcloud_cb(const PointCloud::ConstPtr& msg);

  // the minimum height to use points
  double min_height_;
  // the height above min_height_ in which points will be accumulated
  double range_;

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_subsriber_;
  ros::Publisher pointcloud_publisher_;
  tf::TransformListener tf_listener_;
};


#endif
