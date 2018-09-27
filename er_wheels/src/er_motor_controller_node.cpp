#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "phidgets/motor_encoder.h"

#include <sstream>
#include <cmath>

const int controller_frequency = 25;

const double ticks_per_rev = 3591.84/4;

double omega = 0;

double omega_goal = 0;

// use PI controller, since 25hz is probabaly to low to use d controller in a meaningful way
double p = 0;
double i = 10;

double error_i = 0;

int last_encoder_reading = -1;
ros::Time last_encoder_reading_time;

void angularVelocityCallback(const std_msgs::Float64::ConstPtr& angular_velocity_msg)
{
  omega_goal = angular_velocity_msg->data;
  ROS_DEBUG("omega_goal: [%f]", omega_goal);
}

void encoderCallback(const phidgets::motor_encoder::ConstPtr& encoder_msg)
{
  // it does not seem to be possible to set the publishing frequency of the motor
  // so we decrease the rate by having this callback being called at a slower rate than the motor publishing
  if (last_encoder_reading == -1) {
    last_encoder_reading = encoder_msg->count;
    last_encoder_reading_time = encoder_msg->header.stamp;
  }
  else {
    double delta_t = (encoder_msg->header.stamp-last_encoder_reading_time).toSec();
    omega = 2 * M_PI * (double)(encoder_msg->count - last_encoder_reading) / (delta_t * ticks_per_rev);
    ROS_DEBUG("omega: [%f], T: [%f]", omega, 2*M_PI/omega);
    last_encoder_reading = encoder_msg->count;
    last_encoder_reading_time = encoder_msg->header.stamp;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_controller_node");

  ros::NodeHandle n;

  ros::Publisher pwm_pub = n.advertise<std_msgs::Float32>("/motor_name/cmd_vel", 1);
  ros::Subscriber angular_velocity_sub = n.subscribe("/motor_name/angular_velocity", 1, angularVelocityCallback);
  ros::Subscriber encoder_sub = n.subscribe("/motor_name/encoder", 1, encoderCallback);

  ros::Rate loop_rate(controller_frequency);

  while (ros::ok())
  {
    // get most up-to-date encoder and twist readings
    ros::spinOnce();

    double error = omega_goal - omega;

    error_i += error / controller_frequency;

    // cap i, s.t. final pwm signal is not outside of boundaries
    // this will cap the pwm signial between -100 and 100 as well
    // and prevents i to get very high for unreachable goals and thus
    // taking time to get back up/down
    if(p*error + i*error_i > 100)
      error_i = (100 - p*error) / i;
    else if(p*error + i*error_i < -100)
        error_i = (-100 - p*error) / i;

    ROS_DEBUG("error: %f", error);

    std_msgs::Float32 pwm_msg;
    pwm_msg.data = p*error + i*error_i;

    pwm_pub.publish(pwm_msg);

    loop_rate.sleep();
  }


  return 0;
}
