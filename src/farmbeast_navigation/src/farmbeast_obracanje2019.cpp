#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist CMD_MSG;

float ang_hitrost = 1.0;

int main(int argc, char **argv){
  ros::init(argc, argv, "farmbeast_obracanje2019");
  ros::NodeHandle nh;
  ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  CMD_MSG.angular.z = ang_hitrost;

  ros::Rate r(10);

  while(ros::ok){

    premik.publish(CMD_MSG);
    ros::spinOnce();
  }

  return 0;
}
