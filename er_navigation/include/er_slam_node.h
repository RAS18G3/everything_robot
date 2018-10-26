#ifndef ER_SLAM_NODE_H
#define ER_SLAM_NODE_H

#include "map_reader.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseArray.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <string>
#include <vector>
#include <random>

class SLAMNode {
public:
  SLAMNode();
  ~SLAMNode();

  void run_node();

private:
  // Methods
  void init_node();
  bool reset_localization(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response );
  void publish_particles();

  // Type definitions
  struct Particle {
    double x, y, theta;

    Particle(double pos_x, double pos_y, double pos_theta) : x(pos_x), y(pos_y), theta(pos_theta) {};
  };

  enum State { None, Localization, Tracking };

  // ROS specific member variables
  ros::NodeHandle nh_;
  ros::Publisher map_publisher_;
  ros::Publisher particles_publisher_;
  ros::Rate loop_rate_;
  ros::ServiceServer reset_localization_service_;

  nav_msgs::OccupancyGrid current_map_;

  // Particle filter member variables
  State current_state_;
  std::vector<Particle> particles_;
  int particles_per_m2_;

};


#endif
