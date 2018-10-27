#include "er_slam_node.h"

void fix_angle(double& angle) {
    angle = std::fmod(angle + M_PI, 2*M_PI);
    if (angle < 0)
        angle += M_PI;
    else
      angle -= M_PI;
}

/* evaluate the normal distributions pdf
 * sigma is standard deviation
 */
double evaluate_gaussian(double x, double sigma, double mu=0) {
  return std::exp( - std::pow(x - mu, 2) / (2 * sigma * sigma) ) / (sigma * std::sqrt(2*M_PI));
}

SLAMNode::SLAMNode() : nh_(), loop_rate_(10), current_state_(None), last_odometry_msg_(nullptr) {

}

SLAMNode::~SLAMNode() {

}

void SLAMNode::init_node() {
  std::string map_path;
  std::string odometry_topic;
  std::string laser_scan_topic;
  std::string node_name = ros::this_node::getName();

  // read params
  ros::param::param<std::string>("~map_file", map_path, "");
  ros::param::param<double>("~map_margin", map_margin_, 0.5);
  ros::param::param<int>("~particles_per_m2", particles_per_m2_, 20);
  ros::param::param<int>("~beam_count", beam_count_, 8);
  ros::param::param<std::string>("~odometry_topic", odometry_topic, "/wheel_odometry");
  ros::param::param<std::string>("~laser_scan_topic", laser_scan_topic, "/scan");
  ros::param::param<double>("~alpha_trans_trans", alpha_trans_trans_, 0.001);
  ros::param::param<double>("~alpha_trans_rot", alpha_trans_rot_, 0.00001);
  ros::param::param<double>("~alpha_rot_trans", alpha_rot_trans_, 0.000000001);
  ros::param::param<double>("~alpha_rot_rot", alpha_rot_rot_,     0.000000005);
  ros::param::param<double>("~laser_sigma", laser_sigma_, 0.05);

  MapReader map_reader(map_path);

  map_publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(node_name + "/occupancy_grid", 1);
  particles_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(node_name + "/particles", 1);

  odometry_subscriber_ = nh_.subscribe<nav_msgs::Odometry>(odometry_topic, 1, &SLAMNode::odometry_cb, this);
  laser_scan_subscriber_ = nh_.subscribe<>(laser_scan_topic, 1, &SLAMNode::laser_scan_cb, this);

  current_map_ = map_reader.occupancy_grid(map_margin_);

  // advertise the service which will reset the localization
  reset_localization_service_ = nh_.advertiseService(node_name + "/reset_localization", &SLAMNode::reset_localization_cb, this);

  reset_localization(); // this will init the particle filter
}

void SLAMNode::run_node() {
  init_node();

  while(ros::ok()) {
    ros::spinOnce(); // this will handle service calls and handle all the subscriber callbacks

    map_publisher_.publish(current_map_);
    publish_particles();

    loop_rate_.sleep();

  }
}

bool SLAMNode::reset_localization_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response ) {
  ROS_INFO("Reset localization...");
  response.success = true;

  reset_localization();

  ROS_INFO_STREAM("Reset localization successful. Particles generated: " << particles_.size());
  return true;
}

void SLAMNode::reset_localization() {
  // uniformly initialize the particle set
  particles_.clear();
  // calculate the map area in m^2
  double area = current_map_.info.width * current_map_.info.height * current_map_.info.resolution * current_map_.info.resolution;
  // get a certain number of particles per m^2
  num_particles_ = area * particles_per_m2_;

  double x_min = current_map_.info.origin.position.x;
  double x_max = current_map_.info.origin.position.x + current_map_.info.width * current_map_.info.resolution;
  double y_min = current_map_.info.origin.position.y;
  double y_max = current_map_.info.origin.position.y + current_map_.info.height * current_map_.info.resolution;

  // init random number generators
  std::default_random_engine generator;
  std::uniform_real_distribution<double> x_generator(x_min, x_max);
  std::uniform_real_distribution<double> y_generator(y_min, y_max);
  std::uniform_real_distribution<double> angle_generator(-M_PI, M_PI);
  // generate the particle set
  for(int i=0; i<num_particles_; ++i) {
    particles_.emplace_back(x_generator(generator), y_generator(generator), angle_generator(generator));
  }

  for(auto it = particles_.begin(); it != particles_.end(); ++it) {
    ROS_DEBUG_STREAM(it->x << " " << it->y << " " << it->theta);
  }

  current_state_ = Localization;
}

void SLAMNode::publish_particles() {
  geometry_msgs::PoseArray pose_array_msg;

  // particles are relative to the map
  pose_array_msg.header.frame_id = "/map";

  // convert x,y,theta particles to a full ros pose
  // TODO: maybe its better to store poses directly and operate on them?? not sure about it
  for(auto it = particles_.begin(); it != particles_.end(); ++it) {
    geometry_msgs::Pose pose;
    tf2::Quaternion quaternion_orientation;
    pose.position.x = it->x;
    pose.position.y = it->y;
    quaternion_orientation.setRPY(0,0,it->theta);
    pose.orientation = tf2::toMsg(quaternion_orientation);
    pose_array_msg.poses.push_back(pose);
  }

  particles_publisher_.publish(pose_array_msg);
}


void SLAMNode::odometry_cb(const nav_msgs::Odometry::ConstPtr& msg) {
  // first check if there is a previous odometry msg
  // we will use the position difference to that to calculate the movement between two odometry messages
  if (last_odometry_msg_ != nullptr) {
    // measure time of this callback
    ros::WallTime start_time = ros::WallTime::now();

    // extract 2d position and orientation (where was the robot in previous odometry msg, where is it in the current one)
    // TODO: any nicer way of doing this? probably lots of unnecessary computations done here...
    double x2 = msg->pose.pose.position.x, y2=msg->pose.pose.position.y;
    double x1 = last_odometry_msg_->pose.pose.position.x, y1=last_odometry_msg_->pose.pose.position.y;
    tf2::Quaternion orientation2, orientation1;
    tf2::fromMsg(msg->pose.pose.orientation, orientation2);
    tf2::fromMsg(last_odometry_msg_->pose.pose.orientation, orientation1);
    tf2::Matrix3x3 rotation_matrix2(orientation2);
    tf2::Matrix3x3 rotation_matrix1(orientation1);
    double roll2, pitch2, yaw2, roll1, pitch1, yaw1;
    rotation_matrix2.getRPY(roll2, pitch2, yaw2);
    rotation_matrix1.getRPY(roll1, pitch1, yaw1);

    // algorithm from p. 136, Probabilistic Robotics
    double delta_rot1 = std::atan2(y2-y1, x2-x1) - yaw1;
    double delta_trans = std::sqrt( std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) );
    double delta_rot2 = yaw2 - yaw1 - delta_rot1;

    // create noise generators
    std::default_random_engine generator;
    std::normal_distribution<double> rot1_noise(0,std::sqrt( alpha_rot_rot_ * std::pow(delta_rot1,2) + alpha_trans_rot_ * std::pow(delta_trans,2)));
    std::normal_distribution<double> trans_noise(0,std::sqrt( alpha_rot_trans_ * ( std::pow(delta_rot1,2) + std::pow(delta_rot2,2) ) + alpha_trans_trans_ * std::pow(delta_trans,2)));
    std::normal_distribution<double> rot2_noise(0,std::sqrt( alpha_rot_rot_ * std::pow(delta_rot2,2) + alpha_trans_rot_ * std::pow(delta_trans,2)));

    double delta_rot1_hat, delta_rot2_hat, delta_trans_hat;
    for(auto it = particles_.begin(); it != particles_.end(); ++it) {
      // add different noise for every particle
      delta_rot1_hat = delta_rot1 + rot1_noise(generator);
      delta_trans_hat = delta_trans + trans_noise(generator);
      delta_rot2_hat = delta_rot2 + rot2_noise(generator);

      // update the particle
      it->x = it->x + delta_trans_hat*cos(it->theta + delta_rot1_hat);
      it->y = it->y + delta_trans_hat*sin(it->theta + delta_rot1_hat);
      it->theta = it->theta + delta_rot1_hat + delta_rot2_hat;
      fix_angle(it->theta);
    }

    ROS_INFO_STREAM("Prediction time: " << (ros::WallTime::now()-start_time).toSec());
  }
  last_odometry_msg_ = msg;
}

void SLAMNode::laser_scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
  measurement_update(msg);
}

void SLAMNode::measurement_update(const sensor_msgs::LaserScan::ConstPtr& laser_scan_msg) {
  ros::WallTime start_time = ros::WallTime::now();

  // TEMP FIX
  // TODO: use TF to get the transform between base_link and frame_id of laser_scan_msg and calculate the offset based on that
  const double lidar_angle = M_PI;
  const double lidar_x = 0.03;
  const double lidar_y = 0;

  // first find the laser beams to use
  // the data will be inf in random placces (every 2nd, but sometimes even or odd indices)
  // and we want to maximally use beam_count_ laser beams, optimally equally distributed
  struct LaserScan {
    double range, angle;
    LaserScan(double r, double a) : range(r), angle(a) {};
  };
  std::vector<LaserScan> laser_scans;

  // e.g. if there are 100 beams, and we want 2 beams, this way we will skip 49 indices, and take the next feasible
  // measurement starting at 50 (because we increase the index once more in the for loop)

  int skip = laser_scan_msg->ranges.size() / beam_count_ - 1;
  skip = skip < 0? 0 : skip;

  for(int i = 0; i < laser_scan_msg->ranges.size(); ++i) {
    double range = laser_scan_msg->ranges[i];
    if(!std::isinf(range) && range <= laser_scan_msg->range_max && range >= laser_scan_msg->range_min) {
      double angle =  laser_scan_msg->angle_min + i * laser_scan_msg->angle_increment + lidar_angle;
      fix_angle(angle);
      laser_scans.emplace_back(range, angle);
      i += skip; // we might end up with less than beam_count readings, but only if lots of readings are shitty anyways
    }
  }

  ROS_DEBUG_STREAM("Scan...");
  for(auto it = laser_scans.begin(); it != laser_scans.end(); ++it) {
    ROS_DEBUG_STREAM(it->range << " " << it->angle);
  }

  // for each particle, compare the expected measurement to the actual measurement
  for(auto particles_it = particles_.begin(); particles_it != particles_.end(); ++particles_it) {
    for(auto laser_it = laser_scans.begin(); laser_it != laser_scans.end(); ++laser_it) {
      // offset the particle based on the lidar position
      double x = particles_it->x + lidar_x * cos(particles_it->theta) + lidar_y * sin(particles_it->theta);
      double y = particles_it->y + lidar_x * sin(particles_it->theta) + lidar_y * cos(particles_it->theta);

      // calculate laser angle in global reference frame
      double laser_angle = particles_it->theta + laser_it->angle;
      fix_angle(laser_angle);

      // check what's the expected range
      double range_expected = ray_cast(current_map_, x, y, laser_angle);
      double range_error = laser_it->range - range_expected;

      // adjust the weight of the particle based on how likely that measurement is
      particles_it->weight *= evaluate_gaussian(range_error, laser_sigma_);
    }
  }

  // resample
  resample();
  ROS_INFO_STREAM("Measurement time: " << (ros::WallTime::now()-start_time).toSec());
}

void SLAMNode::resample() {
  // algorithm from p. 110 in Probabilistic Robotics (low variance sampler)

  // normalize the weights, s.t. they add up to 1
  double weight_sum = 0;
  ROS_INFO_STREAM("resampling...");
  for(auto particles_it = particles_.begin(); particles_it != particles_.end(); ++particles_it) {
    weight_sum += particles_it->weight;
  }
  for(auto particles_it = particles_.begin(); particles_it != particles_.end(); ++particles_it) {
    particles_it->weight /= weight_sum;
    ROS_INFO_STREAM(particles_it->weight);
  }

  // using move here, so that there is no unnecessary copy of the particles
  std::vector<Particle> old_particles = std::move(particles_);
  particles_ = std::vector<Particle>();

  // TODO: adaptive resampling
  std::default_random_engine generator;
  std::uniform_real_distribution<double> step_generator(0,  1. / old_particles.size());
  double offset = step_generator(generator);
  double cumulated_weight = old_particles[0].weight, current_weight;
  int current_particle_idx = 0;
  while(particles_.size() < num_particles_) {
    current_weight = offset + (particles_.size()) * 1. / old_particles.size();
    while(current_weight > cumulated_weight) {
      ++current_particle_idx;
      cumulated_weight = cumulated_weight + old_particles[current_particle_idx].weight;
    }
    particles_.push_back(old_particles[current_particle_idx]); // this calls the copy constructor, which will not copy the weight, but resets it to 1
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_slam_node");

  SLAMNode slam_node;

  slam_node.run_node();

  return 0;
}
