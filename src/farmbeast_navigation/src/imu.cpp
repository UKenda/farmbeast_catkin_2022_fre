
#include <sensor_msgs/Imu.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "std_msgs/Int16.h"
#include <vector>
#include <fstream>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

using namespace std;
using namespace geometry_msgs;

#define PI 3.14159265


//CMD_Vel
double roll, pitch, yaw;
bool loop = true, obracanje_k = false, first= true;
int obracanje_z = 0;
float old_angle = 0, new_angle = 0, end_angle = 0, razlika = 0;
//CALLBACK
void Imu(const sensor_msgs::Imu::ConstPtr& imu_msg){
    tf::Quaternion imu_quaternion;
    tf::quaternionMsgToTF(imu_msg->orientation, imu_quaternion);
    tf::Transform imu_orientation(imu_quaternion, tf::Vector3(0,0,0));
    imu_orientation.getBasis().getEulerYPR(yaw, pitch, roll);
    cout << "angle: " << yaw << endl;
    new_angle = yaw; 

}

int main(int argc, char **argv){
    ros::init(argc, argv, "farmbeast_imu_node");
    ros::NodeHandle nh;
    ros::Rate r(10);
    nh.setParam("/farmbeast/imu_node",true);
    
    //sub/pub
    ros::Subscriber laser = nh.subscribe("/imu/data",1, Imu);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    cout << "pred whille" << endl;
    nh.getParam("/farmbeast/imu_node",loop);
    while(loop){
        //loop=false;
        //premik.publish(CMD_MSG);
        //cout << "angle: " << << endl;

        ros::spinOnce();
        
        razlika = razlika + abs(old_angle-new_angle);

        
        nh.getParam("/farmbeast/obracanje",obracanje_z);
        //cout << "Obracanje" << obracanje_z << endl;
        if (obracanje_z!=0){
            old_angle = new_angle;
            if(first){
                cout << "zacetek" << endl;
                first = false;
                razlika = 0;
            }
            geometry_msgs::Twist twist;
            twist.angular.z = 0;
            twist.linear.x = 0;
            premik.publish(twist);
            cout << "razlika" << razlika << endl;
        }
        
        if (razlika > 1.32 && obracanje_z){
            nh.setParam("/farmbeast/obracanje",0);
            first = true;
            cout << "koncna razlika : " << razlika << endl;
            geometry_msgs::Twist twist;
            twist.angular.z = 0;
            twist.linear.x = 0;
            premik.publish(twist);
        }
        r.sleep();

        nh.getParam("/farmbeast/imu_node",loop);
        

    }

    cout << "konec" << endl;

    return 0;
}
