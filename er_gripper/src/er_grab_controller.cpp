#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "geometry_msgs/Twist.h"

int object_num;

double x;
double y;
double width;
double height;

void object_box_callback(const std_msgs::UInt16MultiArray::ConstPtr bounding_box_msg){
  x = bounding_box_msg.data[object_num+0];
  y = bounding_box_msg.data[object_num+1];
  width = bounding_box_msg.data[object_num+2];
  height = bounding_box_msg.data[object_num+3];
}

int main(int argc, char **argv){
  object_num = 0;

  ros::Subscriber object_box_sub = n.subscribe("object_bounding_boxes", 10, object_box_callback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 10);

  while(ros::ok()){
    geometry_msgs::Twist twist_msg;

    double object_point = x + (width/2);
    double middle_point = 300;
    double sideways_treshold = 70;
    double height_treshold = 30;

    if(height < height_treshold){
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0.05*(object_point-middle_point);
    }
    else if (std::abs(object_point-middle_point) < sideways_treshold){
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = 0.1*(object_point-middle_point);
    }
    else{
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0.2*(object_point-middle_point);
    }

    twist_pub.publish(twist_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
