#ifndef ER_OBJECT_FILTER_NODE_H
#define ER_OBJECT_FILTER_NODE_H

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "std_msgs/UInt16MultiArray.h"
#include "er_perception/ObjectList.h"
#include "visualization_msgs/Marker.h"

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

  struct Position2D {
    double x, y;
    Position2D(double xp, double yp) : x(xp), y(yp) {};
  };

  struct ClassifiedBoundingBoxCenter {
    Point2D position;
    int class_id;
    ClassifiedBoundingBoxCenter(int xp, int yp, int cid) : position(xp, yp), class_id(cid) {};
  };
  struct Object {
    Position2D position;
    std::vector<int> class_count;
    int observations;
    int id;
    Object(double xp, double yp, int i) : position(xp, yp), class_count(15), observations(0), id(i) {};
  };


  void init_node();
  void process_data();
  void pointcloud_2d_cb(const PointCloud::ConstPtr& msg);
  void pointcloud_3d_cb(const PointCloud::ConstPtr& msg);
  void boundingbox_cb(const std_msgs::UInt16MultiArray::ConstPtr& msg);
  void publish_objects();

  PointCloud::ConstPtr last_3d_pointcloud_msg_;


  // the number of points inside this radius, s.t. there could be an object in the view
  int threshold_;

  int points_in_camera_;

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_2d_subscriber_;
  ros::Subscriber pointcloud_3d_subscriber_;
  ros::Subscriber boundingbox_subscriber_;
  ros::Publisher object_publisher_;
  ros::Publisher marker_publisher_;
  tf::TransformListener tf_listener_;

  ros::Rate loop_rate_;

  std::vector<ClassifiedBoundingBoxCenter> classified_center_points_;
  std::vector<Object> objects_;

  int id_counter;

  double object_distance_;
};


#endif
