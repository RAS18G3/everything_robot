#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Int16.h"
#include "geometry_msgs/Twist.h"

int object_num;

double x;
double y;
double width;
double height;
bool object_seen;

void object_box_callback(const std_msgs::UInt16MultiArray::ConstPtr bounding_box_msg){
  x = bounding_box_msg -> data[object_num+0];
  y = bounding_box_msg -> data[object_num+1];
  width = bounding_box_msg -> data[object_num+2];
  height = bounding_box_msg -> data[object_num+3];

  object_seen = true;
}

int main(int argc, char **argv){
  object_num = 0;
  object_seen = false;

  ros::init(argc, argv, "er_grab_controller");
  ros::NodeHandle n;

  ros::Subscriber object_box_sub = n.subscribe("object_bounding_boxes", 10, object_box_callback);
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 10);
  //ros::Publisher grip_pub = n.advertise<std_msgs::Int64>("grip", 1);

  ros::Rate loop_rate(25);
  while(ros::ok()){
    geometry_msgs::Twist twist_msg;
    //std_msgs::Int16 grip_msg;

    double object_point = x + (width/2);
    double middle_point = 320;
    double sideways_treshold = 70;
    double height_treshold = 30;

    if(height < height_treshold){
      twist_msg.linear.x = 0;
      twist_msg.angular.z = 0;

      //grip_msg.data = 110;
      //grip_pub.publish(grip_msg);
    }
    else if (std::abs(object_point-middle_point) < sideways_treshold){
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = 0; //0.1*(object_point-middle_point);
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
