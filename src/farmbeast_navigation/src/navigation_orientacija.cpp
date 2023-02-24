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



    //premik
    float xSmer = 0;
    float zSmer;
    bool yPremik = false;

    //IMU
    float oCenter = 0; //tej vrednosti se naj približujemo
    float oMeja = 0.05; //meja natančnosti
    float oMax90 = 0.75; //ko smo obrnjeni za 90 stopinj
    float oMax180 = 0.99;
    float oTrenutna;
    int oStopnja = 1; // 1-gremo naprej, 2-obrnemo se za 90 stopinj, 3-gremo naprej, 4-obrnemo se za 90 stopinj, 5-gremo naprej
    float oCas = 1500; //stevilo milisekund, kako dolgo se bomo peljali
    float oSmer = 1; //ali bomo sli levo ali desno
    float oObrni = 0.15; //kako mocno se bomo obracali

    //stetje vrst
    int sStevec = 0; //štetje vrst
    int sMinS; //minimalni .size() vektorja, ki ga moremo dosečti, da je vrsta zaznan

    //stara/nova
    float stara = 0;
    float nova;

    //CMD_Vel
    geometry_msgs::Twist CMD_MSG;


    //marker
    visualization_msgs::Marker points;
    geometry_msgs::Point p;


void CompasCB(const sensor_msgs::Imu::ConstPtr& compas){
    oTrenutna = compas->orientation.x;

  switch(oStopnja){
    case 1:
    {
       xSmer = 0.2;
       zSmer = 0;
       CMD_MSG.linear.x = xSmer;
       CMD_MSG.angular.z = zSmer;

       break;
    }
    case 2:
    {
       CMD_MSG.linear.x = 0;
       if(oSmer == 1){
           if(oTrenutna > oMax90){
               CMD_MSG.angular.z = oObrni;
           }else
               CMD_MSG.angular.z = 0;
       }else{
           if(oTrenutna < oMax90 * oSmer){
               CMD_MSG.angular.z = -oObrni;
           }else
               CMD_MSG.angular.z = 0;
       }

       break;
    }
    case 3:
    {
      xSmer = 0.2;
      zSmer = 0;
      CMD_MSG.linear.x = xSmer;
      CMD_MSG.angular.z = zSmer;

      break;
    }
    case 4:
    {
      CMD_MSG.linear.x = 0;
      if(oSmer == 1){
          if(oTrenutna > oMax180){
              CMD_MSG.angular.z = oObrni;
          }else
              CMD_MSG.angular.z = 0;
      }else{
          if(oTrenutna < oMax180 * oSmer){
              CMD_MSG.angular.z = -oObrni;
          }else
              CMD_MSG.angular.z = 0;
      }

      break;
    }
    case 5:
    {
      xSmer = 0.2;
      zSmer = 0;
      CMD_MSG.linear.x = xSmer;
      CMD_MSG.angular.z = zSmer;

      break;
    }
    default:
    {
      CMD_MSG.linear.x = 0;
      CMD_MSG.angular.z = 0;
    }
  }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;


    //sub/pub
    ros::Subscriber compas = nh.subscribe("/imu/data", 10, CompasCB);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);


    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;

    while(ros::ok){

        //cout << "Smo not" << endl;
        marker_pub.publish(points);

     switch(oStopnja){
        case 1:
        {
            premik.publish(CMD_MSG);
            oStopnja++;
            usleep(oCas);

            break;
        }
        case 2:
        {
            if(CMD_MSG.angular.z == 0)
                oStopnja++;
            else
                premik.publish(CMD_MSG);

            break;
        }
        case 3:
        {
            premik.publish(CMD_MSG);
            oStopnja++;
            usleep(oCas);

            break;
        }
        case 4:
        {
            if(CMD_MSG.angular.z == 0)
                oStopnja++;
            else
                premik.publish(CMD_MSG);

            break;
        }
        case 5:
        {
            premik.publish(CMD_MSG);
            oStopnja++;
            usleep(oCas);

            break;
        }
        default:
        {
           premik.publish(CMD_MSG);
           return 0;
        }



     }




        ros::spinOnce();
    }


    return 0;
}
