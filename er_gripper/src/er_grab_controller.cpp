#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/Twist.h"

int object_num;

double x;
double y;
double width;
double height;

bool object_seen, object_close, grab_controller_running, done;
int no_bounding_box_counter;
ros::Subscriber object_box_sub;
ros::Publisher twist_pub;
ros::Publisher grab_pub;
ros::ServiceServer service;
int sign(double x) {
  return x>0 ? 1 : -1;
}

bool grab_cb(std_srvs::Trigger::Request  &req,
             std_srvs::Trigger::Response &res)
{
  geometry_msgs::Twist twist_msg;

  ROS_INFO("Trying to grab object...");
  done = false;
  object_close = false;
  grab_controller_running = true;
  no_bounding_box_counter = 0;
  res.success = false;
  while(!done) {
    /*When the camera cant see the object anymore the robot is still to far away to be able to grab the object.
    This following part makes the robot drive a little bit further before it stops when the object has dissapeared from its view.*/

    ros::spinOnce();
    if(object_close == true){
      ROS_INFO("Close in on the object");
      twist_msg.linear.x = 0.1;
      twist_pub.publish(twist_msg);
      ros::Duration(1).sleep();

      twist_msg.linear.x = 0;
      twist_pub.publish(twist_msg);

      std_msgs::Int64 angle_msg;
      angle_msg.data = 110;
      grab_pub.publish(angle_msg);
      ros::Duration(1).sleep();
      res.success = true;
      done = true;
    }
  }
  grab_controller_running = false;
  return true;
}

void object_box_callback(const std_msgs::UInt16MultiArray::ConstPtr bounding_box_msg){
  geometry_msgs::Twist twist_msg;
  if(grab_controller_running) {
    if(bounding_box_msg->data.size() == 0) {
      ROS_INFO("No object in view");
      if(++no_bounding_box_counter > 5) {
        done = true;
        twist_pub.publish(twist_msg);
        ROS_INFO("Stop, there is no object");
        
      }
      return;
    }

    //gets data from bounding_box_msg
    x = bounding_box_msg -> data[object_num+0];
    y = bounding_box_msg -> data[object_num+1];
    width = bounding_box_msg -> data[object_num+2];
    height = bounding_box_msg -> data[object_num+3];

    //calculates middle of object and sets tresholds
    double object_point = x + (width/2);
    double middle_point = 320; // the image is 640 wide wich makes 320 the middle on the x-axis
    double sideways_treshold = 65;
    double height_treshold = 300;


    if(y > height_treshold){
      //if the bounding box has a low height it means it is close and we should stop
      ROS_INFO("height_treshold");
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
      object_seen = false;
      object_close = true;
      twist_pub.publish(twist_msg);
    }
    else if (std::abs(object_point-middle_point) < sideways_treshold){
      //the object is within an angle so we can drive straigh forward
      ROS_INFO("sideways_treshold");
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = 0.4*0.01*(middle_point-object_point);
      object_seen = true; //keeps track that we have detected and tries to catch an object
      object_close = false;
      twist_pub.publish(twist_msg);
    }
    else{
      //the object is close to the side of the image wich means we need to spin to get it to the middle
      ROS_INFO("spinning");
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0.5*sign(middle_point-object_point);
      object_seen = true; //keeps track that we have detected and tries to catch an object
      object_close = false;
      twist_pub.publish(twist_msg);
      ros::Duration(0.3).sleep();
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;
      twist_pub.publish(twist_msg);
      ros::Duration(0.5).sleep();
    }

  }
}

int main(int argc, char **argv){
  object_num = 0; //0 means we want to catch the first object we see (if several)

  ros::init(argc, argv, "er_grab_controller");
  ros::NodeHandle n;

  object_box_sub = n.subscribe("object_bounding_boxes", 1, object_box_callback);
  twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 1);
  grab_pub = n.advertise<std_msgs::Int64>("grip", 1);
  service = n.advertiseService("start_grip", &grab_cb);
  grab_controller_running = false;

  ros::spin();
}
