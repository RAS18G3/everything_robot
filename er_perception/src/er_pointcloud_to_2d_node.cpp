#include "er_pointcloud_to_2d_node.h"

PointCloudTo2DNode::PointCloudTo2DNode() : nh_() {

}

PointCloudTo2DNode::~PointCloudTo2DNode() {

}

void PointCloudTo2DNode::pointcloud_cb(const PointCloud::ConstPtr& msg) {
  PointCloud transformed_pointcloud;
  PointCloud modified_pointcloud;
  PointCloud map_pointcloud;

  // transform the point cloud such that ground is parallel to the floor
  pcl_ros::transformPointCloud("/base_link", pcl_conversions::fromPCL(msg->header.stamp), *msg, "/camera_depth_frame", transformed_pointcloud, tf_listener_);
  pcl_ros::transformPointCloud("/map", pcl_conversions::fromPCL(msg->header.stamp), *msg, "/camera_depth_frame", map_pointcloud, tf_listener_);


  for(auto it=map_pointcloud.begin(); it != map_pointcloud.end(); ++it) {
    if(!std::isnan(it->x)) {
      if(it->x >= safearea_xmin_-0.01 && it->x <= safearea_xmax_+0.01 && it->y <= safearea_ymax_+0.01 && it->y >= safearea_ymin_-0.01) {
        for(auto it=transformed_pointcloud.begin(); it != transformed_pointcloud.end(); ++it) {
          if(!std::isnan(it->x)) {
            if(it-> z > min_height_ && it->z < min_height_+range_ && std::sqrt(std::pow(it->x,2)+std::pow(it->y,2)) < 0.6) {
              it->z = 0;
              modified_pointcloud.push_back(*it);
              break;
            }
          }
        }
        break;
      }
      else {
        // remove all nan entrise (which correspond to now point), and threshold based on the defined height
        for(auto it=transformed_pointcloud.begin(); it != transformed_pointcloud.end(); ++it) {
          if(!std::isnan(it->x)) {
            if(it-> z > min_height_ && it->z < min_height_+range_ && std::sqrt(std::pow(it->x,2)+std::pow(it->y,2)) < 0.6) {
              it->z = 0;
              modified_pointcloud.push_back(*it);
            }
          }
        }
        break;
      }
    }
  }


  // set the reference frame
  // this could also be map/odom, but this way no transform is required to get the distance from this points to the robot
  modified_pointcloud.header.frame_id = "/base_link";
  modified_pointcloud.header.stamp = msg->header.stamp;
  pointcloud_publisher_.publish(modified_pointcloud);
}

void PointCloudTo2DNode::init_node() {
  std::string camera_pointcloud_topic;
  std::string out_pointcloud_topic;
  ros::param::param<double>("~min_height", min_height_, 0);
  ros::param::param<double>("~range", range_, 0.01);
  ros::param::param<double>("/safe_area/x_min", safearea_xmin_, 0.0);
  ros::param::param<double>("/safe_area/x_max", safearea_xmax_, 0.5);
  ros::param::param<double>("/safe_area/y_min", safearea_ymin_, 0.0);
  ros::param::param<double>("/safe_area/y_max", safearea_ymax_, 0.5);
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
