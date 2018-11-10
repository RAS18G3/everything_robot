#include "er_object_filter_node.h"

ObjectFilterNode::ObjectFilterNode() : nh_(),  last_3d_pointcloud_msg_(nullptr), loop_rate_(10), id_counter(0) {

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
  std::string node_name = ros::this_node::getName();
  std::string pointcloud_2d_topic; // assuming this is relative to the robots base_link
  std::string pointcloud_3d_topic;
  std::string boundingbox_topic;
  ros::param::param<int>("~threshold", threshold_, 100);
  ros::param::param<double>("~object_distance", object_distance_, 0.1);
  ros::param::param<std::string>("~pointcloud_3d_topic", pointcloud_3d_topic, "/camera/depth_registered/points");
  ros::param::param<std::string>("~pointcloud_2d_topic", pointcloud_2d_topic, "/camera/pointcloud_2d");
  ros::param::param<std::string>("~classified_object_bounding_boxes_topic", boundingbox_topic, "/object_bounding_boxes_classified");

  pointcloud_2d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_2d_topic, 1, &ObjectFilterNode::pointcloud_2d_cb, this);
  pointcloud_3d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_3d_topic, 1, &ObjectFilterNode::pointcloud_3d_cb, this);
  boundingbox_subscriber_ = nh_.subscribe<std_msgs::UInt16MultiArray>(boundingbox_topic, 1, &ObjectFilterNode::boundingbox_cb, this);
  object_publisher_ = nh_.advertise<er_perception::ObjectList>("/objects", 1);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>( "/object_markers", 0 );

  reset_objects_service_ = nh_.advertiseService(node_name + "/reset_objects", &ObjectFilterNode::reset_objects_cb, this);
  remove_object_service_ = nh_.advertiseService(node_name + "/remove_object", &ObjectFilterNode::remove_object_cb, this);
}

void ObjectFilterNode::process_data() {
  PointCloud transformed_pointcloud, filtered_pointcloud;
  const int box_size = 10; // box_size x box_size pixels will be checked and averaged over
  if(points_in_camera_ > threshold_ && last_3d_pointcloud_msg_ != nullptr) {
    // PointCloud transformed_pointcloud;last_3last_3d_pointcloud_msg_

    for(auto point_it=classified_center_points_.begin(); point_it!=classified_center_points_.end(); ++point_it) {
      // loop over box around center point of bounding box to check the 3d position
      // TODO: check if this could be outside of the image (should not happen in our case because bounding boxes have minimum sizes in x and y)

      int count = 0;
      double x_avg=0, y_avg = 0, z_avg = 0;
      int class_id = point_it->class_id;
      for(int img_x = point_it->position.x - box_size / 2; img_x < point_it->position.x + box_size / 2; ++img_x) {
        for(int img_y = point_it->position.y - box_size / 2; img_y < point_it->position.y + box_size / 2; ++img_y) {
          double x = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].x;
          double y = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].y;
          double z = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].z;
          if(!std::isnan(x)) {
            x_avg += x;
            y_avg += y;
            z_avg += z;
            ++count;
          }
        }
      }

     ROS_DEBUG_STREAM("count" << count);
      // check if there are a few 3d points to calculate the position
      if(count > 5) {
        ROS_DEBUG("count >5");
        // calculate the average map position of these points
        x_avg /= count;
        y_avg /= count;
        z_avg /= count;

        // only transform the average point to the map frame
        filtered_pointcloud.header.frame_id = "/camera_rgb_optical_frame";
        filtered_pointcloud.push_back(pcl::PointXYZ(x_avg, y_avg, z_avg));
        pcl_ros::transformPointCloud("/map", pcl_conversions::fromPCL(last_3d_pointcloud_msg_->header.stamp), filtered_pointcloud, "/camera_rgb_optical_frame", transformed_pointcloud, tf_listener_);

        x_avg = transformed_pointcloud[0].x;
        y_avg = transformed_pointcloud[0].y;

        // check if there is a similar object in the map already
        bool new_object = true;
        for(auto object_it = objects_.begin(); object_it != objects_.end(); ++object_it) {
          // ROS_INFO_STREAM("Distance " << std::sqrt(std::pow(object_it->position.x - x_avg, 2) + std::pow(object_it->position.y - y_avg, 2)) << " " << x_avg << " " << y_avg << " " << object_it->position.x << " " << object_it->position.y);
          if(std::sqrt(std::pow(object_it->position.x - x_avg, 2) + std::pow(object_it->position.y - y_avg, 2)) < object_distance_) {
            new_object = false;
            object_it->position.x = (object_it->observations * object_it->position.x + x_avg) / (object_it->observations + 1);
            object_it->position.y = (object_it->observations * object_it->position.y + y_avg) / (object_it->observations + 1);
            ++object_it->class_count[class_id];
            ++object_it->observations;
            int most_likely_class = std::max_element(object_it->class_count.begin(), object_it->class_count.end()) - object_it->class_count.begin();
            ROS_DEBUG_STREAM("Old object: " << object_it->position.x << " " << object_it->position.y << " " << most_likely_class);
            ROS_DEBUG_STREAM("Seeing old object id " << object_it - objects_.begin());
            break;
          }
        }
        if(new_object) {
          objects_.emplace_back(x_avg, y_avg, id_counter++);
          ++objects_.back().class_count.at(class_id);
          objects_.back().observations = 1;
          ROS_INFO_STREAM("New object: " << x_avg << " " << y_avg << " " << class_id << ", Total objects: " << objects_.size());
        }
      }
    }
  }
}

bool ObjectFilterNode::reset_objects_cb(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response ) {
  objects_.clear();

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "objects";
  marker.action = visualization_msgs::Marker::DELETEALL;
  marker_publisher_.publish(marker);

  response.success = true;
  return true;
}

bool ObjectFilterNode::remove_object_cb( er_perception::RemoveObject::Request& request, er_perception::RemoveObject::Response& response ) {
  for(auto objects_it = objects_.begin(); objects_it!=objects_.end(); ++objects_it ) {
    if(objects_it->id == request.id) {
      response.success = true;
      objects_.erase(objects_it);

      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "objects";
      marker.action = visualization_msgs::Marker::DELETE;
      marker.id = request.id;
      marker_publisher_.publish(marker);
      return true;
    }
  }
  response.success = false;
  return false;
}

void ObjectFilterNode::publish_objects() {
  er_perception::ObjectList objects_msg;
  for(auto object_it = objects_.begin(); object_it != objects_.end(); ++object_it) {
    er_perception::Object new_object;
    new_object.x = object_it->position.x;
    new_object.y = object_it->position.y;
    new_object.class_id = std::max_element(object_it->class_count.begin(), object_it->class_count.end()) - object_it->class_count.begin() ;
    new_object.id = object_it->id;
    objects_msg.objects.push_back(new_object);
  }
  object_publisher_.publish(objects_msg);

  // publish rviz markers
  for(auto object_it = objects_.begin(); object_it != objects_.end(); ++object_it) {
    double x = object_it->position.x;
    double y = object_it->position.y;
    int class_id = std::max_element(object_it->class_count.begin(), object_it->class_count.end()) - object_it->class_count.begin();
    int id = object_it->id;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "objects";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.orientation.w = 1.0;

    // LABELS = ['Yellow Ball', 'Yellow Cube', 'Green Cube', 'Green Cylinder', 'Green Hollow Cube', 'Orange Cross', 'Patric', 'Red Cylinder', 'Red Hollow Cube', 'Red Ball', 'Blue Cube', 'Blue Triangle', 'Purple Cross', 'Purple Star', 'Other']

    switch(class_id) {
      case 0: // yellow ball
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.color.r = 1.0;
        marker.color.g = 0.95;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 1: // yellow cube
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 1.0;
        marker.color.g = 0.95;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 2: // green cube
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 3: // green cube
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        break;
      case 4: // green hollow cube
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0.0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        break;
      case 5: // orange cross
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 1;
        marker.color.g = 0.4;
        marker.color.b = 0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 6: // patric (orange star)
        marker.type = visualization_msgs::Marker::ARROW;
        marker.color.r = 1;
        marker.color.g = 0.4;
        marker.color.b = 0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 7: // red cylinder
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        break;
      case 8: // red hollow cube
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        break;
      case 9: // red ball
        marker.type = visualization_msgs::Marker::SPHERE;;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 10: // blue cube
        marker.type = visualization_msgs::Marker::CUBE;;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 11: // blue triangle
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 12: // purple cross
        marker.type = visualization_msgs::Marker::CUBE;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
      case 13: // purple star
        marker.type = visualization_msgs::Marker::ARROW;;
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        break;
    }
    marker.scale.x = 0.04;
    marker.scale.y = 0.04;
    marker.scale.z = 0.04;
    marker_publisher_.publish( marker );

  }
}

void ObjectFilterNode::run_node() {
  init_node();

  while(ros::ok()) {
    // get most up to date data
    ros::spinOnce();

    // fuse data
    process_data();

    publish_objects();

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
