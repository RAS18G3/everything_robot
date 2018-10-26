#include "er_slam_node.h"

SLAMNode::SLAMNode() : nh_(), loop_rate_(10), current_state_(None) {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::init_node() {
  std::string map_path;
  std::string node_name = ros::this_node::getName();
  ros::param::param<std::string>("~map_file", map_path, "");

  MapReader map_reader(map_path);

  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

  current_map_ = map_reader.occupancy_grid();

  // advertise the reset_localization service
  reset_localization_service_ = nh_.advertiseService(node_name + "/reset_localization", &SLAMNode::reset_localization, this);
}

void SLAMNode::run_node() {
  init_node();

  while(ros::ok()) {
    map_publisher_.publish(current_map_);

    loop_rate_.sleep();
  }
}

bool SLAMNode::reset_localization(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response ) {

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
