#ifndef ER_OBJECT_FILTER_NODE_H
#define ER_OBJECT_FILTER_NODE_H

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "std_msgs/UInt16MultiArray.h"

#include <string>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

// TODO: add lidar subscriber to remove walls from 2d point cloud

class ObjectFilterNode {
public:
  ObjectFilterNode();
  ~ObjectFilterNode();

  void run_node();

private:
  struct Point2D {
    int x, y;
    Point2D(int xp, int yp) : x(xp), y(yp) {};
  };


  void init_node();
  void process_data();
  void pointcloud_2d_cb(const PointCloud::ConstPtr& msg);
  void pointcloud_3d_cb(const PointCloud::ConstPtr& msg);
  void boundingbox_cb(const std_msgs::UInt16MultiArray::ConstPtr& msg);

  PointCloud::ConstPtr last_3d_pointcloud_msg_;


  // the number of points inside this radius, s.t. there could be an object in the view
  int threshold_;

  int points_in_camera_;

  ros::Rate loop_rate_;

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_2d_subscriber_;
  ros::Subscriber pointcloud_3d_subscriber_;
  ros::Subscriber boundingbox_subscriber_;
  tf::TransformListener tf_listener_;

  std::vector<Point2D> center_points_;
};


#endif
