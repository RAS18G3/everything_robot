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

ros::Publisher twist_pub;

int sign(double x) {
  return x>0 ? 1 : -1;
}

void object_box_callback(const std_msgs::UInt16MultiArray::ConstPtr bounding_box_msg){
  x = bounding_box_msg -> data[object_num+0];
  y = bounding_box_msg -> data[object_num+1];
  width = bounding_box_msg -> data[object_num+2];
  height = bounding_box_msg -> data[object_num+3];

  double object_point = x + (width/2);
  double middle_point = 320;
  double sideways_treshold = 80;
  double height_treshold = 25;

  geometry_msgs::Twist twist_msg;

  if(height < height_treshold){
    ROS_INFO("height_treshold");
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0;
  }
  else if (std::abs(object_point-middle_point) < sideways_treshold){
    ROS_INFO("sideways_treshold");
    twist_msg.linear.x = 0.15;
    twist_msg.angular.z = 0; //001*(middle_point-object_point);
  }
  else{
    ROS_INFO("spinning");
    twist_msg.linear.x = 0;
    twist_msg.angular.z = 0.004*(middle_point-object_point);
  }

twist_pub.publish(twist_msg);
object_seen = true;
}

int main(int argc, char **argv){
  object_num = 0;

  ros::init(argc, argv, "er_grab_controller");
  ros::NodeHandle n;

  ros::Subscriber object_box_sub = n.subscribe("object_bounding_boxes", 10, object_box_callback);
  twist_pub = n.advertise<geometry_msgs::Twist>("/cartesian_motor_controller/twist", 10);
  //ros::Publisher grab_pub = n.advertise<std_msgs::Int64>("grip", 10);

  geometry_msgs::Twist twist_msg;
  twist_pub.publish(twist_msg);

  ros::Rate loop_rate(25);
  while(ros::ok()){
    ros::spinOnce();
    if(object_seen == false){
      twist_msg.linear.x = 0.1;
      for(int i = 0; i < 25; i++){
        twist_pub.publish(twist_msg);
        ros::spinOnce();
      }
      twist_msg.linear.x = 0;
      twist_pub.publish(twist_msg);

      //std_msgs::Int64 angle_msg;
      //angle_msg.data = 110;
      //grab_pub.publish(angle_msg);

    }
    object_seen = false;
    loop_rate.sleep();

}
}
