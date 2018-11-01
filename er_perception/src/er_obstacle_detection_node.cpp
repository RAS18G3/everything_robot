#include "er_obstacle_detection_node.h"

ObstacleDetectionNode::ObstacleDetectionNode() : nh_() {

}

ObstacleDetectionNode::~ObstacleDetectionNode() {

}

void ObstacleDetectionNode::pointcloud_cb(const PointCloud::ConstPtr& msg) {
  int in_radius_count = 0;
  double range_squared = std::pow(range_, 2);

  // count how many points are inside the defined radius
  for(auto it=msg->begin(); it != msg->end(); ++it) {
    if(std::pow(it->x,2) + std::pow(it->y, 2) < range_squared ) {
      ++in_radius_count;
    }
  }

  // publish whether or not there are more points than threshold_ inside the radius
  std_msgs::Bool bool_msg;
  bool_msg.data = in_radius_count >= threshold_ ? true : false;
  obstacle_publisher_.publish(bool_msg);

  ROS_DEBUG("inliers: %d", in_radius_count);
}

void ObstacleDetectionNode::init_node() {
  std::string pointcloud_2d_topic; // assuming this is relative to the robots base_link
  std::string obstacle_topic;
  ros::param::param<double>("~range", range_, 0);
  ros::param::param<int>("~threshold", threshold_, 0.01);
  ros::param::param<std::string>("~obstacle_topic", obstacle_topic, "/obstacle");
  ros::param::param<std::string>("~pointcloud_2d_topic", pointcloud_2d_topic, "/camera/pointcloud_2d");


  pointcloud_subsriber_ = nh_.subscribe<PointCloud>(pointcloud_2d_topic, 1, &ObstacleDetectionNode::pointcloud_cb, this);
  obstacle_publisher_ = nh_.advertise<std_msgs::Bool>(obstacle_topic, 1);
}

void ObstacleDetectionNode::run_node() {
  init_node();

  ros::spin();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_obstacle_detection_node");

  ObstacleDetectionNode obstacle_detection_node;

  obstacle_detection_node.run_node();

  return 0;
}
