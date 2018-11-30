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
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "pcl_ros/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.h"

#include "er_navigation/MapLoadSave.h"

#include <rosbag/bag.h>
#include "rosbag/view.h"

#include <string>
#include <vector>
#include <random>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class SLAMNode {
public:
  SLAMNode();
  ~SLAMNode();

  void run_node();

private:
  // Methods
  void init_node();
  bool reset_localization_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response );
  bool save_cb(er_navigation::MapLoadSave::Request& request, er_navigation::MapLoadSave::Response& response  );
  bool load_cb(er_navigation::MapLoadSave::Request& request, er_navigation::MapLoadSave::Response& response );
  void reset_localization();
  void publish_particles();
  void publish_transform();
  void odometry_cb(const nav_msgs::Odometry::ConstPtr& msg);
  void laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
  void pointcloud_cb(const PointCloud::ConstPtr& msg);
  bool motion_update();
  void measurement_update();
  void resample();
  void map_update();
  bool save_map(std::string name);
  bool load_map(std::string name);

  // Type definitions
  struct Particle {
    double x, y, theta, weight;

    Particle(double pos_x, double pos_y, double pos_theta) : x(pos_x), y(pos_y), theta(pos_theta), weight(1) {};
    Particle(const Particle& rhs ) : x(rhs.x), y(rhs.y), theta(rhs.theta), weight(1) {};
  };

  enum State { None, Localization, Tracking };
  struct LaserScan {
    double range, angle;
    LaserScan(double r, double a) : range(r), angle(a) {};
  };

  // ROS specific member variables
  ros::NodeHandle nh_;
  ros::Publisher map_publisher_;
  ros::Publisher particles_publisher_;
  ros::Subscriber odometry_subscriber_;
  ros::Subscriber laser_scan_subscriber_;
  ros::Subscriber pointcloud_subscriber_;
  ros::Rate loop_rate_;
  ros::ServiceServer reset_localization_service_;
  ros::ServiceServer load_service_;
  ros::ServiceServer save_service_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;
  tf2_ros::Buffer transform_buffer_;
  tf2_ros::TransformListener transform_listener_;
  tf::TransformListener tf_listener_;

  nav_msgs::OccupancyGrid current_map_;
  nav_msgs::OccupancyGrid current_lidar_map_;
  nav_msgs::OccupancyGrid current_obstacle_map_;
  geometry_msgs::TransformStamped current_odomotry_msg_;
  geometry_msgs::TransformStamped last_odometry_msg_;
  sensor_msgs::LaserScan::ConstPtr current_laser_scan_msg_;
  PointCloud::ConstPtr current_point_cloud_msg_;

  MapReader map_reader_;

  ros::Time update_step_time_;

  // Particle filter member variables
  State current_state_; // this will keep track of whether the robot is localized, in which case mapping can be done
  std::vector<Particle> particles_;
  double map_margin_;
  int particles_per_m2_; // to calculate how many particles to generate based on map size
  int beam_count_; // to reduce the computational load, this is the maximum number of beams to be used
  int num_particles_; // the number of particles after resetting and as long as the algorithm is not adaptive
  double alpha_rot_rot_, alpha_rot_trans_, alpha_trans_rot_, alpha_trans_trans_;
  double gaussian_pos_, gaussian_theta_;
  double laser_sigma_;
  double tracking_threshold_;
  double x_est_, y_est_, yaw_est_;
  int tracking_particles_;
  double camera_fov_;
  double camera_range_;
  double safearea_xmin_, safearea_xmax_, safearea_ymin_, safearea_ymax_;

};


#endif
