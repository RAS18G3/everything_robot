#include "er_pointcloud_to_2d_node.h"

PointCloudTo2DNode::PointCloudTo2DNode() : nh_() {

}

PointCloudTo2DNode::~PointCloudTo2DNode() {

}

void PointCloudTo2DNode::pointcloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  ROS_INFO("Received pointcloud");

  sensor_msgs::PointCloud2 transformed_pointcloud;



  pcl_ros::transformPointCloud("/map", *msg, transformed_pointcloud, tf_listener_);
  transformed_pointcloud.height = transformed_pointcloud.height * transformed_pointcloud.width;
  transformed_pointcloud.width = 1;

  pointcloud_publisher_.publish(transformed_pointcloud);
}

void PointCloudTo2DNode::init_node() {
  std::string camera_pointcloud_topic;
  std::string out_pointcloud_topic;
  ros::param::param<double>("~min_height", min_height_, 0);
  ros::param::param<double>("~range", range_, 0.01);
  ros::param::param<std::string>("~camera_pointcloud_topic", camera_pointcloud_topic, "/camera/depth/points");
  ros::param::param<std::string>("~out_pointcloud_topic", out_pointcloud_topic, "/camera/pointcloud_2d");


  pointcloud_subsriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(camera_pointcloud_topic, 1, &PointCloudTo2DNode::pointcloud_cb, this);
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(out_pointcloud_topic, 1);
}

void PointCloudTo2DNode::run_node() {
  init_node();

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  PointCloudTo2DNode pointcloud_to_2d_node;

  pointcloud_to_2d_node.run_node();

  return 0;
}
