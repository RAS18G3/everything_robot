#include "er_slam_node.h"

SLAMNode::SLAMNode() : nh_(), loop_rate_(10), current_state_(None) {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::init_node() {
  std::string map_path;
  std::string node_name = ros::this_node::getName();

  // read params
  ros::param::param<std::string>("~map_file", map_path, "");
  ros::param::param<int>("~particles_per_m2", particles_per_m2_, 20);

  MapReader map_reader(map_path);

  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>("/occupancy_grid", 1);

  current_map_ = map_reader.occupancy_grid();

  // advertise the service which will reset the localization
  reset_localization_service_ = nh_.advertiseService(node_name + "/reset_localization", &SLAMNode::reset_localization, this);
}

void SLAMNode::run_node() {
  init_node();

  while(ros::ok()) {
    ros::spinOnce();

    map_publisher_.publish(current_map_);
    loop_rate_.sleep();

  }
}

bool SLAMNode::reset_localization(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response ) {
  ROS_INFO("Reset localization...");
  response.success = true;

  // uniformly initialize the particle set
  particles_.clear();
  // calculate the map area in m^2
  double area = current_map_.info.width * current_map_.info.height * current_map_.info.resolution * current_map_.info.resolution;
  // get a certain number of particles per m^2
  int num_particles = area * particles_per_m2_;

  double x_min = current_map_.info.origin.position.x;
  double x_max = current_map_.info.origin.position.x + current_map_.info.width * current_map_.info.resolution;
  double y_min = current_map_.info.origin.position.y;
  double y_max = current_map_.info.origin.position.y + current_map_.info.height * current_map_.info.resolution;

  std::default_random_engine generator;
  std::uniform_real_distribution<double> x_generator(x_min, x_max);
  std::uniform_real_distribution<double> y_generator(y_min, y_max);
  std::uniform_real_distribution<double> angle_generator(-M_PI, M_PI);
  for(int i=0; i<num_particles; ++i) {
    particles_.emplace_back(x_generator(generator), y_generator(generator), angle_generator(generator));
  }

  for(auto it = particles_.begin(); it != particles_.end(); ++it) {
    ROS_DEBUG_STREAM(it->x << " " << it->y << " " << it->theta);
  }

  current_state_ = Localization;

  ROS_INFO_STREAM("Reset localization successful. Particles generated: " << particles_.size());
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
