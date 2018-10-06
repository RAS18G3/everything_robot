#include "er_slam_node.h"

SLAMNode::SLAMNode() {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::run_node() {
  std::string map_path;
  ros::param::param<std::string>("~map_file", map_path, "");

  MapReader map_reader(map_path);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
