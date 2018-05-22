#include "ros/ros.h"
#include <iostream>

//msg includes
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
  //Ros node initializations
  ros::init(argc, argv, "globalplanner");
  ros::NodeHandle nh;

  double velocity;

  //default parameter setting
  nh.param<double>("target_velocity", velocity, 0.0 );

  //Velocity Publisher object
  ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("global_plan", 1);

  if(nh.getParam("target_velocity", velocity))
  {
    ROS_INFO("Fetched updated target velocity");
  }

  //std::cout << "Velocity: " << velocity << std::endl;

  int count;
  ros::Rate rate(100);
  while(ros::ok())
  {
    vel_pub.publish(velocity);

    ros::spinOnce();
    rate.sleep();
    count++;
  }
  double stop_velocity;
  stop_velocity = 0;
  std::cout << "n Loop: " << count << std::endl;

  std::cout << "Velocity: " << stop_velocity << std::endl;

  vel_pub.publish(stop_velocity);

  ros::spinOnce();

  //shutdown node
  ros::shutdown();
}//main
