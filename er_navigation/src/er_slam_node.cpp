#include "er_slam_node.h"

SLAMNode::SLAMNode() : nh_(), loop_rate_(10) {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::init_node() {
  std::string map_path;
  ros::param::param<std::string>("~map_file", map_path, "");

  MapReader map_reader(map_path);

  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

  current_map_ = map_reader.occupancy_grid();
}

void SLAMNode::run_node() {
  init_node();

  while(ros::ok()) {
    map_publisher_.publish(current_map_);

    loop_rate_.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
