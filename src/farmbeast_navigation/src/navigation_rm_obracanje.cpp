#include <sensor_msgs/LaserScan.h>
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
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace geometry_msgs;

#define PI 3.14159265

//koliko stopinj bomo dejansko prebrali na vsaki strani
int st_mer = 30;
float hitrost = 0.1;
float koeficient;

float obrni = -1;
int farmbeast_task_select=1;
bool robot_run = false;
bool ustavi = false;

bool leva_niT = false;
bool desna_niT = false;
int ack_nac = 0;

//povprecenje
vector<float> cos_kot;
vector<float> arctan_kot;
float value;
float napaka = 0.02;
float alfa;


//leva stran
vector<float> mer_L;
vector<float> kot_L;
float kaL; //koeficient premice L
float naL; //n premice L
float xsumL, x2sumL, ysumL, xysumL;


//desna stran
vector<float> mer_D;
vector<float> kot_D;
float kaD; //koeficient premice D
float naD; //n premice D
float xsumD, x2sumD, ysumD, xysumD;

//premik
float xSmer = 0;
float zSmer;
bool yPremik = false;

//delo s premicami
float min_val = 0.05;
float max_val = 0.60;
float max_obr = 850;
float xOdmik;
float yOdmik;
float yOdmik_sredina; //treba vnesti, ko stestiramo
float yMax; //največja razlika med koefientoma, TREBA NASTAVITI

float povpL = 0;
float povpD = 0;
float absPovpL = 0;
float absPovpD = 0;

//stara/nova
float stara = 0;
float nova = 0;


//CMD_Vel
geometry_msgs::Twist CMD_MSG;


//marker
visualization_msgs::Marker points;
geometry_msgs::Point p;

//CALLBACK
void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){
    cout << "test 1 2 3 4" << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    cout << "zdravo!" << endl;
    int stevec_hercov = 0;
    int ack_nac = 0;

	nh.setParam("drive_mode_skeed",false);
	nh.setParam("drive_mode_ackerman",true);







    if(nh.hasParam("/PhidgetsImuNodelet/hitrost_z")){
        nh.getParam("/PhidgetsImuNodelet/hitrost", hitrost);
        nh.getParam("/PhidgetsImuNodelet/hitrost_z", zSmer);
        nh.getParam("/PhidgetsImuNodelet/ack_nacin", ack_nac);
    }
    nh.setParam("ackerman_drive_mode",ack_nac);

    koeficient = hitrost*koeficient;


    //sub/pub
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber laser = nh.subscribe("/scan",10, LaserCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;

    double rate = 10.0;
    ros::Rate r(rate);



    while(ros::ok){

        marker_pub.publish(points);

        if(!ustavi){
            CMD_MSG.angular.z = zSmer;
            CMD_MSG.linear.x = hitrost;
            premik.publish(CMD_MSG);
            cout << "objavljamo" << endl;
            stevec_hercov++;
            if(stevec_hercov == 30){
                ustavi = true;
            }
        }else{
            CMD_MSG.angular.z = 0;
            CMD_MSG.linear.x = 0;
            premik.publish(CMD_MSG);
            break;
        }

        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
