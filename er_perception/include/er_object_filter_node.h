#ifndef ER_OBJECT_FILTER_NODE_H
#define ER_OBJECT_FILTER_NODE_H

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "std_msgs/UInt16MultiArray.h"
#include "er_perception/ObjectList.h"
#include "er_perception/ClassifiedImage.h"
#include "er_perception/RemoveObject.h"
#include "visualization_msgs/Marker.h"
#include "std_srvs/Trigger.h"
#include "ras_msgs/RAS_Evidence.h"
#include "std_msgs/String.h"


#include <string>
#include <cmath>
#include <vector>

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

  struct ObjectCandidate {
    Position2D position;
    int class_id;
    ObjectCandidate(double xp, double yp, int cid) : position(xp, yp), class_id(cid) {};
  };
  struct Object {
    Position2D position;
    std::vector<int> class_count;
    int observations;
    int id;
    int class_id;
    bool evidence_published;
    Object(double xp, double yp, int i) : position(xp, yp), class_count(15), observations(0), id(i), evidence_published(false) {};
  };


  void init_node();
  void process_data();
  void pointcloud_2d_cb(const PointCloud::ConstPtr& msg);
  void pointcloud_3d_cb(const PointCloud::ConstPtr& msg);
  void boundingbox_cb(const er_perception::ClassifiedImage::ConstPtr& msg);
  bool reset_objects_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response );
  bool remove_object_cb(er_perception::RemoveObject::Request& request, er_perception::RemoveObject::Response& response );
  void publish_objects();
  void handle_object(double x, double y, int class_id);
  void mergeObjects(double x, double y, int class_id, int object_id);

  PointCloud::ConstPtr last_3d_pointcloud_msg_;
  er_perception::ClassifiedImage::ConstPtr last_classified_image_msg_;


  // the number of points inside this radius, s.t. there could be an object in the view
  int threshold_;

  int points_in_camera_;

  ros::NodeHandle nh_;
  ros::Subscriber pointcloud_2d_subscriber_;
  ros::Subscriber pointcloud_3d_subscriber_;
  ros::Subscriber boundingbox_subscriber_;
  ros::Publisher object_publisher_;
  ros::Publisher marker_publisher_;
  ros::Publisher evidence_publisher_;
  ros::Publisher  speak_publisher_;
  ros::ServiceServer reset_objects_service_;
  ros::ServiceServer remove_object_service_;
  tf::TransformListener tf_listener_;

  ros::Rate loop_rate_;

  std::vector<ClassifiedBoundingBoxCenter> classified_center_points_;
  std::vector<ObjectCandidate> positioned_center_points_;
  std::vector<Object> objects_;

  int id_counter;

  double object_distance_;
  double same_object_distance_;

  int required_observations_;

};


#endif
