#ifndef ER_SLAM_NODE_H
#define ER_SLAM_NODE_H

#include "map_reader.h"
#include "occupancy_grid_utils.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
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
  void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
  void measurement_update(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg);

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
  ros::Subscriber odometry_subscriber_;
  ros::Rate loop_rate_;
  ros::ServiceServer reset_localization_service_;

  nav_msgs::OccupancyGrid current_map_;
  nav_msgs::Odometry::ConstPtr last_odometry_msg_;

  // Particle filter member variables
  State current_state_; // this will keep track of whether the robot is localized, in which case mapping can be done
  std::vector<Particle> particles_;
  double map_margin_;
  int particles_per_m2_; // to calculate how many particles to generate based on map size
  int beam_count_; // to reduce the computational load, this is the maximum number of beams to be used
  double alpha_rot_rot_, alpha_rot_trans_, alpha_trans_rot_, alpha_trans_trans_;

};


#endif
