#include "er_slam_node.h"

SLAMNode::SLAMNode() : nh_() {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::run_node() {
  std::string map_path;
  ros::param::param<std::string>("~map_file", map_path, "");

  MapReader map_reader(map_path);

  ros::Publisher map_publisher = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

  ros::Rate loop_rate(10);

  while(ros::ok()) {
    map_publisher.publish(map_reader.occupancy_grid());

    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
