#include "er_object_filter_node.h"

ObjectFilterNode::ObjectFilterNode() : nh_(), loop_rate_(10), last_3d_pointcloud_msg_(nullptr) {

}

ObjectFilterNode::~ObjectFilterNode() {

}

void ObjectFilterNode::pointcloud_2d_cb(const PointCloud::ConstPtr& msg) {
  points_in_camera_ = msg->size();

  ROS_DEBUG("inliers: %d", points_in_camera_);
}

void ObjectFilterNode::pointcloud_3d_cb(const PointCloud::ConstPtr& msg) {
  last_3d_pointcloud_msg_ = msg;

  ROS_DEBUG("received 3d pointcloud");
}


void ObjectFilterNode::boundingbox_cb(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
  int count=0;
  center_points_.clear();
  int x, y, width, height;
  for(auto it = msg->data.begin(); it != msg->data.end(); ++it) {
    switch(count) {
      case 0:
        x = *it;
        break;
      case 1:
        y = *it;
        break;
      case 2:
        width = *it;
        break;
      case 3:
        height = *it;
        center_points_.emplace_back(x+width/2, y+height/2);
        break;
      default:
        ROS_ERROR("this should not happen.");
        break;
    }
    count = (count++)%4;
  }
}

void ObjectFilterNode::init_node() {
  std::string pointcloud_2d_topic; // assuming this is relative to the robots base_link
  std::string pointcloud_3d_topic;
  std::string boundingbox_topic;
  ros::param::param<int>("~threshold", threshold_, 100);
  ros::param::param<std::string>("~pointcloud_3d_topic", pointcloud_3d_topic, "/camera/depth/points");
  ros::param::param<std::string>("~pointcloud_2d_topic", pointcloud_2d_topic, "/camera/pointcloud_2d");
  ros::param::param<std::string>("~object_bounding_boxes_topic", boundingbox_topic, "/object_bounding_boxes");

  pointcloud_2d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_2d_topic, 1, &ObjectFilterNode::pointcloud_2d_cb, this);
  pointcloud_3d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_3d_topic, 1, &ObjectFilterNode::pointcloud_3d_cb, this);
  boundingbox_subscriber_ = nh_.subscribe<std_msgs::UInt16MultiArray>(boundingbox_topic, 1, &ObjectFilterNode::boundingbox_cb, this);
}

void ObjectFilterNode::process_data() {
  if(points_in_camera_ > threshold_ && last_3d_pointcloud_msg_ != nullptr) {
    // PointCloud transformed_pointcloud;
    // pcl_ros::transformPointCloud("/base_link", *msg, last_3last_3d_pointcloud_msg_, tf_listener_);
    for(auto point_it=center_points_.begin(); point_it!=center_points_.end(); ++point_it) {
      ROS_INFO("%d %d -> %f %f %f", point_it->x, point_it->y, (*last_3d_pointcloud_msg_)[point_it->y * 640 + point_it->x].x,  (*last_3d_pointcloud_msg_)[point_it->y * 640 + point_it->x].y,  (*last_3d_pointcloud_msg_)[point_it->y * 640 + point_it->x].z);
    }
  }
}

void ObjectFilterNode::run_node() {
  init_node();

  while(ros::ok()) {
    // get most up to date data
    ros::spinOnce();

    // fuse data
    process_data();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_obstacle_filter_node");

  ObjectFilterNode obstacle_filter_node;

  obstacle_filter_node.run_node();

  return 0;
}
