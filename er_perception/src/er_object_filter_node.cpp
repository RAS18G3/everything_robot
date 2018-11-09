#include "er_object_filter_node.h"

ObjectFilterNode::ObjectFilterNode() : nh_(),  last_3d_pointcloud_msg_(nullptr), loop_rate_(10) {

}

ObjectFilterNode::~ObjectFilterNode() {

}

void ObjectFilterNode::pointcloud_2d_cb(const PointCloud::ConstPtr& msg) {
  points_in_camera_ = msg->size();

  ROS_DEBUG("inliers: %d", points_in_camera_);
}

void ObjectFilterNode::pointcloud_3d_cb(const PointCloud::ConstPtr& msg) {
  last_3d_pointcloud_msg_ = msg;

  ROS_DEBUG("received 3d pointcloud");
}


void ObjectFilterNode::boundingbox_cb(const std_msgs::UInt16MultiArray::ConstPtr& msg) {
  int count=0;
  ROS_DEBUG("received bounding boxes");
  classified_center_points_.clear();
  int x, y, width, height, class_id;
  for(auto it = msg->data.begin(); it != msg->data.end(); ++it) {
    switch(count) {
      case 0:
        x = *it;
        break;
      case 1:
        y = *it;
        break;
      case 2:
        width = *it;
        break;
      case 3:
        height = *it;
        break;
      case 4:
        class_id = *it;
        classified_center_points_.emplace_back(x+width/2, y+height/2, class_id);
        ROS_DEBUG("add point");
        break;
      default:
        ROS_ERROR("this should not happen.");
        break;
    }
    count = (++count)%5;
  }
}

void ObjectFilterNode::init_node() {
  std::string pointcloud_2d_topic; // assuming this is relative to the robots base_link
  std::string pointcloud_3d_topic;
  std::string boundingbox_topic;
  ros::param::param<int>("~threshold", threshold_, 100);
  ros::param::param<double>("~object_distance", object_distance_, 0.2);
  ros::param::param<std::string>("~pointcloud_3d_topic", pointcloud_3d_topic, "/camera/depth_registered/points");
  ros::param::param<std::string>("~pointcloud_2d_topic", pointcloud_2d_topic, "/camera/pointcloud_2d");
  ros::param::param<std::string>("~classified_object_bounding_boxes_topic", boundingbox_topic, "/object_bounding_boxes_classified");

  pointcloud_2d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_2d_topic, 1, &ObjectFilterNode::pointcloud_2d_cb, this);
  pointcloud_3d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_3d_topic, 1, &ObjectFilterNode::pointcloud_3d_cb, this);
  boundingbox_subscriber_ = nh_.subscribe<std_msgs::UInt16MultiArray>(boundingbox_topic, 1, &ObjectFilterNode::boundingbox_cb, this);
}

void ObjectFilterNode::process_data() {
  PointCloud transformed_pointcloud;
  const int box_size = 20; // box_size x box_size pixels will be checked and averaged over
  if(points_in_camera_ > threshold_ && last_3d_pointcloud_msg_ != nullptr) {
    // PointCloud transformed_pointcloud;last_3last_3d_pointcloud_msg_
    pcl_ros::transformPointCloud("/map", pcl_conversions::fromPCL(last_3d_pointcloud_msg_->header.stamp), *last_3d_pointcloud_msg_, "/camera_rgb_optical_frame", transformed_pointcloud, tf_listener_);

    for(auto point_it=classified_center_points_.begin(); point_it!=classified_center_points_.end(); ++point_it) {
      // loop over box around center point of bounding box to check the 3d position
      // TODO: check if this could be outside of the image (should not happen in our case because bounding boxes have minimum sizes in x and y)

      int count = 0;
      double x_avg=0, y_avg = 0;
      int class_id = point_it->class_id;
      for(int img_x = point_it->position.x - box_size / 2; img_x < point_it->position.x + box_size / 2; ++img_x) {
        for(int img_y = point_it->position.y - box_size / 2; img_y < point_it->position.y + box_size / 2; ++img_y) {
          double x = transformed_pointcloud[img_y * 640 + img_x].x;
          double y = transformed_pointcloud[img_y * 640 + img_x].y;
          double z = transformed_pointcloud[img_y * 640 + img_x].z;
          if(!std::isnan(x)) {
            x_avg += x;
            y_avg += y;
            ++count;
          }
        }
      }

     ROS_INFO_STREAM("count" << count);
      // check if there are a few 3d points to calculate the position
      if(count > 5) {
        ROS_DEBUG("count >5");
        // calculate the average map position of these points
        x_avg /= count;
        y_avg /= count;

        // check if there is a similar object in the map already
        bool new_object = true;
        for(auto object_it = objects_.begin(); object_it != objects_.end(); ++object_it) {
          ROS_INFO_STREAM("Distance " << std::sqrt(std::pow(object_it->position.x - x_avg, 2) + std::pow(object_it->position.y - y_avg, 2)) << " " << x_avg << " " << y_avg << " " << object_it->position.x << " " << object_it->position.y);
          if(std::sqrt(std::pow(object_it->position.x - x_avg, 2) + std::pow(object_it->position.y - y_avg, 2)) < object_distance_) {
            new_object = false;
            objects_.back().position.x = (objects_.back().observations * objects_.back().position.x + x_avg) / (objects_.back().observations + 1);
            objects_.back().position.y = (objects_.back().observations * objects_.back().position.y + y_avg) / (objects_.back().observations + 1);
            ++objects_.back().class_count.at(class_id);
            ++objects_.back().observations;
            int most_likely_class = std::max_element(objects_.back().class_count.begin(), objects_.back().class_count.end()) - objects_.back().class_count.begin();
            ROS_DEBUG_STREAM("Old object: " << objects_.back().position.x << " " << objects_.back().position.y << " " << most_likely_class);
            ROS_INFO_STREAM("Seeing old object id " << object_it - objects_.begin());
            break;
          }
        }
        if(new_object) {
          objects_.emplace_back(x_avg, y_avg);
          ++objects_.back().class_count.at(class_id);
          objects_.back().observations = 1;
          ROS_INFO_STREAM("New object: " << x_avg << " " << y_avg << " " << class_id << ", Total objects: " << objects_.size());
        }
      }
    }
  }
}

void ObjectFilterNode::run_node() {
  init_node();

  while(ros::ok()) {
    // get most up to date data
    ros::spinOnce();

    // fuse data
    process_data();

    loop_rate_.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "er_obstacle_filter_node");

  ObjectFilterNode obstacle_filter_node;

  obstacle_filter_node.run_node();

  return 0;
}
