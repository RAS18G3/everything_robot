#include "er_pointcloud_to_2d_node.h"

PointCloudTo2DNode::PointCloudTo2DNode() : nh_() {

}

PointCloudTo2DNode::~PointCloudTo2DNode() {

}

void PointCloudTo2DNode::pointcloud_cb(const PointCloud::ConstPtr& msg) {
  PointCloud transformed_pointcloud;
  PointCloud modified_pointcloud;

  // transform the point cloud such that ground is parallel to the floor
  pcl_ros::transformPointCloud("/base_link", *msg, transformed_pointcloud, tf_listener_);

  // remove all nan entrise (which correspond to now point), and threshold based on the defined height
  for(auto it=transformed_pointcloud.begin(); it != transformed_pointcloud.end(); ++it) {
    if(!std::isnan(it->x)) {
      if(it-> z > min_height_ && it->z < min_height_+range_) {
        it->z = 0;
        modified_pointcloud.push_back(*it);
      }
    }
  }

  // set the reference frame
  // this could also be map/odom, but this way no transform is required to get the distance from this points to the robot
  modified_pointcloud.header.frame_id = "/base_link";
  pointcloud_publisher_.publish(modified_pointcloud);
}

void PointCloudTo2DNode::init_node() {
  std::string camera_pointcloud_topic;
  std::string out_pointcloud_topic;
  ros::param::param<double>("~min_height", min_height_, 0);
  ros::param::param<double>("~range", range_, 0.01);
  ros::param::param<std::string>("~camera_pointcloud_topic", camera_pointcloud_topic, "/camera/depth/points");
  ros::param::param<std::string>("~out_pointcloud_topic", out_pointcloud_topic, "/camera/pointcloud_2d");


  pointcloud_subsriber_ = nh_.subscribe<PointCloud>(camera_pointcloud_topic, 1, &PointCloudTo2DNode::pointcloud_cb, this);
  pointcloud_publisher_ = nh_.advertise<PointCloud>(out_pointcloud_topic, 1);
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
