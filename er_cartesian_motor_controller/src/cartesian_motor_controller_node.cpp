#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

const double pi = 3.1415;

const int controller_frequency = 100;

const int ticks_per_rev = 360;
const double b = 0.115; // distance from wheel to base
const double r = 0.0352; // radius of the wheels
const int encoder_frequency = 10; // frequency of encoder readings

double v_l = 0; // velocity left wheel
double v_r = 0; // velocity right wheel

double v_l_goal = 0;
double v_r_goal = 0;

// use PI controller, since 10 hz is probabaly to low to use d controller in a meaningful way
double p = 300;
double i = 600;

double error_i_l = 0;
double error_i_r = 0;

void twistCallback(const geometry_msgs::Twist::ConstPtr& twist_msg)
{
  v_l_goal = twist_msg->linear.x - b * twist_msg->angular.z;
  v_r_goal = twist_msg->linear.x + b * twist_msg->angular.z;

  ROS_INFO("v_l_goal: [%f]", v_l_goal);
  ROS_INFO("v_r_goal: [%f]", v_r_goal);
}

void encoderCallback(const ras_lab1_msgs::Encoders::ConstPtr& encoders_msg)
{
  v_l = 2 * pi * r * encoder_frequency * encoders_msg->delta_encoder1 / ticks_per_rev;
  v_r = 2 * pi * r * encoder_frequency * encoders_msg->delta_encoder2 / ticks_per_rev;

  ROS_INFO("v_l: [%f]", v_l);
  ROS_INFO("v_r: [%f]", v_r);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");

  ros::NodeHandle n;

  ros::Publisher pwm_pub = n.advertise<ras_lab1_msgs::PWM>("/motor_name/pwm", 1);
  ros::Subscriber twist_sub = n.subscribe("/motor_controller/twist", 1, twistCallback);
  ros::Subscriber encoder_sub = n.subscribe("/motor_name/encoder", 1, encoderCallback);


  ros::Rate loop_rate(controller_frequency);

  while (ros::ok())
  {
    // get most up-to-date encoder and twist readings
    ros::spinOnce();

    double error_l = v_l_goal - v_l;
    double error_r = v_r_goal - v_r;

    error_i_l += error_l / controller_frequency;
    error_i_r += error_r / controller_frequency;

    // cap i, s.t. final pwm signal is not outside of boundaries
    // this will cap the pwm signial between -255 and 255 as well
    // and prevents i to get very high for unreachable goals and thus
    // taking time to get back up/down
    if(p*error_l + i*error_i_l > 100)
      error_i_l = (100 - p*error_l) / i;
    else if(p*error_l + i*error_i_l < -100)
        error_i_l = (-100 - p*error_l) / i;

      if(p*error_r + i*error_i_r > 100)
        error_i_r = (100 - p*error_r) / i;
      else if(p*error_r + i*error_i_r < -100)
          error_i_r = (-100 - p*error_r) / i;

    ras_lab1_msgs::PWM pwm_msg;
    pwm_msg.PWM1 = p*error_l + i*error_i_l;
    pwm_msg.PWM2 = p*error_r + i*error_i_r;

    pwm_pub.publish(pwm_msg);

    loop_rate.sleep();
  }


  return 0;
}
