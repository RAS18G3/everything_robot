#include "er_object_filter_node.h"

bool similar_objects(int class_1, int class_2) {
    // will return true for same colored class ids, and false different colored classes
    if( ( class_1 == 0 || class_1 == 1 ) && ( class_2 == 0 || class_2 == 1 ) )
        return true;
    if( ( class_1 == 2 || class_1 == 3 || class_1 == 4 ) && ( class_2 == 2 || class_2 == 3 || class_2 == 4 ) )
        return true;
    if( ( class_1 == 5 || class_1 == 6 ) && ( class_2 == 5 || class_2 == 6 ) )
        return true;
    if( ( class_1 == 7 || class_1 == 8 || class_1 == 9 ) && ( class_2 == 7 || class_2 == 8 || class_2 == 9 ) )
        return true;
    if( ( class_1 == 10 || class_1 == 11 ) && ( class_2 == 10 || class_2 == 11 ) )
        return true;
    if( ( class_1 == 12 || class_1 == 13 ) && ( class_2 == 12 || class_2 == 13 ) )
        return true;
    return false;
}

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

void ObjectFilterNode::boundingbox_cb(const er_perception::ClassifiedImage::ConstPtr& msg) {
  int count=0;
  ROS_DEBUG("received bounding boxes");
  classified_center_points_.clear();
  positioned_center_points_.clear();
  int x, y, width, height, class_id;

  // ROS_INFO_STREAM("delay " << (ros::Time::now() - msg->image.header.stamp).toSec());
  for(auto it = msg->bounding_boxes.begin(); it != msg->bounding_boxes.end(); ++it) {
    x = it->x;
    y = it->y;
    width = it->w;
    height = it->h;
    class_id = it->class_id;
    // classified_center_points_.emplace_back(x+width/2, y+height/2, class_id);
    // ROS_DEBUG("add point");



    // use intrinsics to get position relative to base_link
    double alpha =(x + width/2.0 - 313.0) / 616.;
    double beta =(y + 0.9*height - 239.0) / 616.;
    double camera_angle = 0.5;
    double height = 0.133;
    double camera_offset_y = 0.09;
    double y_test = (std::cos(camera_angle)*height - beta*std::sin(camera_angle)*height) / ( beta * std::cos(camera_angle) + std::sin(camera_angle) );
    double x_test = alpha * (std::sin(camera_angle)*height + std::cos(camera_angle) * y_test);
    y_test += camera_offset_y;


    double x_baselink = y_test;
    double y_baselink = -x_test;
    // ROS_INFO_STREAM(x_baselink << " " << y_baselink);


    PointCloud transformed_pointcloud, pointcloud;

    pointcloud.push_back(pcl::PointXYZ(x_baselink, y_baselink, 0));

    pointcloud.header.frame_id = "/base_link";


      // ROS_INFO_STREAM( x_avg << " " << positioned_center_points_[count_counter].x << " " << y_avg << " " <<  positioned_center_points_[count_counter].y );
    pcl_ros::transformPointCloud("/map", msg->image.header.stamp, pointcloud, "/base_link", transformed_pointcloud, tf_listener_);

    // ROS_INFO_STREAM(transformed_pointcloud[0].x << " " << transformed_pointcloud[0].y);

    positioned_center_points_.emplace_back(transformed_pointcloud[0].x, transformed_pointcloud[0].y, class_id);

    last_classified_image_msg_ = msg;
  }

}

void ObjectFilterNode::init_node() {
  std::string node_name = ros::this_node::getName();
  std::string pointcloud_2d_topic; // assuming this is relative to the robots base_link
  std::string pointcloud_3d_topic;
  std::string boundingbox_topic;
  ros::param::param<int>("~threshold", threshold_, 100);
  ros::param::param<double>("~object_distance", object_distance_, 0.03);
  ros::param::param<double>("~same_object_distance", same_object_distance_, 0.2);
  ros::param::param<std::string>("~pointcloud_3d_topic", pointcloud_3d_topic, "/camera/depth_registered/points");
  ros::param::param<std::string>("~pointcloud_2d_topic", pointcloud_2d_topic, "/camera/pointcloud_2d");
  ros::param::param<std::string>("~classified_object_bounding_boxes_topic", boundingbox_topic, "/object_bounding_boxes_classified");

  // pointcloud_2d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_2d_topic, 1, &ObjectFilterNode::pointcloud_2d_cb, this);
  // pointcloud_3d_subscriber_ = nh_.subscribe<PointCloud>(pointcloud_3d_topic, 1, &ObjectFilterNode::pointcloud_3d_cb, this);
  boundingbox_subscriber_ = nh_.subscribe<er_perception::ClassifiedImage>(boundingbox_topic, 1, &ObjectFilterNode::boundingbox_cb, this);
  object_publisher_ = nh_.advertise<er_perception::ObjectList>("/objects", 1);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>( "/object_markers", 0 );
  evidence_publisher_ = nh_.advertise<ras_msgs::RAS_Evidence>("/evidence", 100);
  speak_publisher_ = nh_.advertise<std_msgs::String>("/espeak/string", 1);

  reset_objects_service_ = nh_.advertiseService(node_name + "/reset_objects", &ObjectFilterNode::reset_objects_cb, this);
  remove_object_service_ = nh_.advertiseService(node_name + "/remove_object", &ObjectFilterNode::remove_object_cb, this);
}

void ObjectFilterNode::process_data() {


  // PointCloud transformed_pointcloud, filtered_pointcloud;
  // const int box_size = 10; // box_size x box_size pixels will be checked and averaged over
  // if(points_in_camera_ > threshold_ && last_3d_pointcloud_msg_ != nullptr) {
    // PointCloud transformed_pointcloud;last_3last_3d_pointcloud_msg_

  int count_counter = 0;
  // ROS_INFO_STREAM("size" << positioned_center_points_.size());

  for(auto point_it=positioned_center_points_.begin(); point_it!=positioned_center_points_.end(); ++point_it) {
    // loop over box around center point of bounding box to check the 3d position
    // TODO: check if this could be outside of the image (should not happen in our case because bounding boxes have minimum sizes in x and y)

    // int count = 0;
    // double x_avg=0, y_avg = 0, z_avg = 0;
    // int class_id = point_it->class_id;
    // for(int img_x = point_it->position.x - box_size / 2; img_x < point_it->position.x + box_size / 2; ++img_x) {
    //   for(int img_y = point_it->position.y - box_size / 2; img_y < point_it->position.y + box_size / 2; ++img_y) {
    //     double x = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].x;
    //     double y = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].y;
    //     double z = (*last_3d_pointcloud_msg_)[img_y * 640 + img_x].z;
    //     if(!std::isnan(x)) {
    //       x_avg += x;
    //       y_avg += y;
    //       z_avg += z;
    //       ++count;
    //     }
    //   }
    // }

   // ROS_DEBUG_STREAM("count" << count);
    // check if there are a few 3d points to calculate the position
    // if(count > 5) {
      // ROS_DEBUG("count >5");
      // calculate the average map position of these points
      // x_avg /= count;
      // y_avg /= count;
      // z_avg /= count;

      // only transform the average point to the map frame
      // filtered_pointcloud.header.frame_id = "/camera_rgb_optical_frame";
      // filtered_pointcloud.push_back(pcl::PointXYZ(x_avg, y_avg, z_avg));
      // pcl_ros::transformPointCloud("/map", pcl_conversions::fromPCL(last_3d_pointcloud_msg_->header.stamp), filtered_pointcloud, "/camera_rgb_optical_frame", transformed_pointcloud, tf_listener_);

      // x_avg = transformed_pointcloud[0].x;
      // y_avg = transformed_pointcloud[0].y;

    handle_object(positioned_center_points_[count_counter].position.x, positioned_center_points_[count_counter].position.y, positioned_center_points_[count_counter].class_id); // update the current object list (x, y should be in the map frame)

    ++count_counter;
    // }
  }
  // }
}

void ObjectFilterNode::handle_object(double x, double y, int class_id) {
  // check if there is a similar object in the map already
  bool new_object = true;
  // ROS_INFO_STREAM(x << " " << y << " " << class_id);
  for(auto object_it = objects_.begin(); object_it != objects_.end(); ++object_it) {
    // ROS_INFO_STREAM("Distance " << std::sqrt(std::pow(object_it->position.x - x_avg, 2) + std::pow(object_it->position.y - y_avg, 2)) << " " << x_avg << " " << y_avg << " " << object_it->position.x << " " << object_it->position.y);
    if((std::sqrt(std::pow(object_it->position.x - x, 2) + std::pow(object_it->position.y - y, 2)) < same_object_distance_ && similar_objects(object_it->class_id, class_id)) ||
        std::sqrt(std::pow(object_it->position.x - x, 2) + std::pow(object_it->position.y - y, 2)) < object_distance_) {
      new_object = false;
      object_it->position.x = 0.95 * object_it->position.x + 0.05 * x;
      object_it->position.y = 0.95 * object_it->position.y + 0.05 * y;
      ++object_it->class_count[class_id];
      ++object_it->observations;
      int most_likely_class = std::max_element(object_it->class_count.begin(), object_it->class_count.end()) - object_it->class_count.begin();
      object_it->class_id = most_likely_class;
      ROS_INFO_STREAM("Old object: " << object_it->position.x << " " << object_it->position.y << " " << most_likely_class);
      ROS_INFO_STREAM("Seeing old object id " << object_it - objects_.begin());

      if(object_it->observations >= 10 and object_it->evidence_published == false) {
        object_it->evidence_published = true;
        ras_msgs::RAS_Evidence evidence_msg;
        //HERE
        std_msgs::String speak_msg;
        evidence_msg.stamp = ros::Time::now();
        evidence_msg.group_number = 3;
        evidence_msg.object_location.transform.translation.x = object_it->position.x;
        evidence_msg.object_location.transform.translation.y = object_it->position.y;
        evidence_msg.image_evidence = last_classified_image_msg_->image;
        switch(most_likely_class) {
          case 0: // yellow ball
            evidence_msg.object_id = evidence_msg.yellow_ball;
            speak_msg.data = std::string("I see a yellow ball");
            break;
          case 1: // yellow cube
            evidence_msg.object_id = evidence_msg.yellow_cube;
            speak_msg.data = std::string("I see a yellow cube");
            break;
          case 2: // green cube
            evidence_msg.object_id = evidence_msg.green_cube;
            speak_msg.data = std::string("I see a green cube");
            break;
          case 3: // green cylinder
            evidence_msg.object_id = evidence_msg.green_cylinder;
            speak_msg.data = std::string("I see a green cylinder");
            break;
          case 4: // green hollow cube
            evidence_msg.object_id = evidence_msg.green_hollow_cube;
            speak_msg.data = std::string("I see a green hollow cube");
            break;
          case 5: // orange cross
            evidence_msg.object_id = evidence_msg.orange_cross;
            speak_msg.data = std::string("I see an orange cross");
            break;
          case 6: // patric (orange star)
            evidence_msg.object_id = evidence_msg.patric;
            speak_msg.data = std::string("I see Patric the Starfish");
            break;
          case 7: // red cylinder
            evidence_msg.object_id = evidence_msg.red_cylinder;
            speak_msg.data = std::string("I see a red cylinder");
            break;
          case 8: // red hollow cube
            evidence_msg.object_id = evidence_msg.red_hollow_cube;
            speak_msg.data = std::string("I see a red hollow cube");
            break;
          case 9: // red ball
            evidence_msg.object_id = evidence_msg.red_ball;
            speak_msg.data = std::string("I see a red ball");
            break;
          case 10: // blue cube
            evidence_msg.object_id = evidence_msg.blue_cube;
            speak_msg.data = std::string("I see a blue cube");
            break;
          case 11: // blue triangle
            evidence_msg.object_id = evidence_msg.blue_triangle;
            speak_msg.data = std::string("I see a blue triangle");
            break;
          case 12: // purple cross
            evidence_msg.object_id = evidence_msg.purple_cross;
            speak_msg.data = std::string("I see a purple cross");
            break;
          case 13: // purple star
            evidence_msg.object_id = evidence_msg.purple_star;
            speak_msg.data = std::string("I see a purple star");
            break;
        }
        evidence_publisher_.publish(evidence_msg);
        //HERE
        ROS_INFO("I am about to pub speak_msg:");
        speak_publisher_.publish(speak_msg);
      }
      break;
    }
  }
  if(new_object) {
    objects_.emplace_back(x, y, id_counter++);
    ++objects_.back().class_count.at(class_id);
    objects_.back().observations = 1;
    ROS_INFO_STREAM("New object: " << x << " " << y << " " << class_id << ", Total objects: " << objects_.size());
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
