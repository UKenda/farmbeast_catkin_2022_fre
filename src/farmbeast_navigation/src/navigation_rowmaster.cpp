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




    float obrni = -1;
    int farmbeast_task_select=1;
    bool robot_run = false;

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

    //IMU
    float oCenter = -0.2; //tej vrednosti se naj približujemo
    float oMeja = 0.05; //meja natančnosti
    float oMax = 0.57; //ko smo obrnjeni za 90 stopinj
    float o180 = oMax*2; //centru pristejemo/odstejemo to vrednost ko pridemo v novo vrsto
    float oTrenutna;

    //delo s premicami
    float min_val = 0.05;
    float max_val = 1.60;
    float xOdmik;
    float yOdmik;
    float yOdmik_sredina; //treba vnesti, ko stestiramo
    float yMax; //največja razlika med koefientoma, TREBA NASTAVITI



    //stara/nova
    float stara = 0;
    float nova = 0;

    bool l1 = false; //če sta l1 in l2 true, potem smo prišli na konec vrste
    bool l2 = false;

    //CMD_Vel
    geometry_msgs::Twist CMD_MSG;


    //marker
    visualization_msgs::Marker points;
    geometry_msgs::Point p;


void CompasCB(const sensor_msgs::Imu::ConstPtr& compas){
    oTrenutna = compas->orientation.x;
    //cout << oTrenutna << endl;
}

void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){

   // cout << "callback" << endl;

    points.DELETEALL;
    // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;





        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;



        //skozi vrsto

            //leva premica

            for(int l = 45; l <= 75; l++){
                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                    kot_L.push_back(l - 45); //pretvorba kota za levo stran
                }
            }

            xsumL = 0; x2sumL = 0; ysumL = 0; xysumL = 0;


            for(int i = 0; i < mer_L.size(); i++){
                xsumL = xsumL + mer_L[i];
                ysumL = ysumL + kot_L[i];
                x2sumL = x2sumL + pow(mer_L[i],2);
                xysumL = xysumL + mer_L[i]*kot_L[i];
            }

            kaL = (mer_L.size()*xysumL-xsumL*ysumL)/(mer_L.size()*x2sumL-xsumL*xsumL); //koeficient
            naL = (x2sumL*ysumL-xsumL*xysumL)/(x2sumL*mer_L.size()-xsumL*xsumL);



            //desna premica
            for(int l = 225; l >= 195; l--){

                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_D.push_back(scan->ranges[l]);
                    kot_D.push_back(225 - l); //pretvorba kota za desno stran
                }
            }

            xsumD = 0; x2sumD = 0; ysumD = 0; xysumD = 0;


            for(int i = 0; i < mer_D.size(); i++){
                xsumD = xsumD + mer_D[i];
                ysumD = ysumD + kot_D[i];
                x2sumD = x2sumD + pow(mer_D[i],2);
                xysumD = xysumD + mer_D[i]*kot_D[i];
            }

            kaD = (mer_D.size()*xysumD-xsumD*ysumD)/(mer_D.size()*x2sumD-xsumD*xsumD); //koeficient
            naD = (x2sumD*ysumD-xsumD*xysumD)/(x2sumD*mer_D.size()-xsumD*xsumD);

            xOdmik = ((naD - naL)/(kaL - kaD)); //x koordinata
            //cout << xOdmik << endl;
            yOdmik = kaL*xOdmik + naL; //y koordinata

            zSmer = xOdmik * -1;
            while(zSmer > 1 || zSmer < -1){
                zSmer = zSmer / 10;
            }

            if(zSmer > 0){
                    if(zSmer < 0.05)
                        zSmer = 0;
                    else if(zSmer < 0.1)
                        zSmer = zSmer;
                    else if(zSmer < 0.15)
                        zSmer = 0.15;
                    else if(zSmer < 0.2)
                        zSmer = 0.2;
                    else if(zSmer < 0.3)
                        zSmer = 0.3;
                    else if(zSmer < 0.4)
                        zSmer = 0.4;
                    else if(zSmer < 0.5)
                        zSmer = 0.5;
                    else if(zSmer < 0.7)
                        zSmer = 0.7;
                    else if(zSmer < 0.9)
                        zSmer = 0.9;
                    else
                        zSmer = 1;
            }else{
                    if(zSmer > -0.05)
                        zSmer = 0;
                    else if(zSmer > -0.1)
                        zSmer = zSmer;
                    else if(zSmer > -0.15)
                        zSmer = -0.15;
                    else if(zSmer > -0.2)
                        zSmer = -0.2;
                    else if(zSmer > -0.3)
                        zSmer = -0.3;
                    else if(zSmer > -0.4)
                        zSmer = -0.4;
                    else if(zSmer > -0.5)
                        zSmer = -0.5;
                    else if(zSmer > -0.7)
                        zSmer = -0.7;
                    else if(zSmer > -0.9)
                        zSmer = -0.9;
                    else
                        zSmer = -1;
            }
/*
            if((yOdmik < yOdmik_sredina + 10.0) && (yOdmik > yOdmik_sredina - 10.0)){
                    zSmer = -1 * ((abs(kaL)-abs(kaD))*0.6)/yMax;
                    yPremik = true;
                    cout << "Tu smo" << endl;
            }
*/
            nova = obrni*zSmer;
            //cout << nova << " " << zSmer << endl;
            mer_L.clear();
            mer_D.clear();
            kot_L.clear();
            kot_D.clear();
    //}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    cout << "zdravo " << endl;
    int rate = 10.0;
    ros::Rate r(rate);


    //sub/pub
    //ros::Subscriber compas = nh.subscribe("/imu/data", 10, CompasCB);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber laser = nh.subscribe("/scan",10, LaserCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;

    while(ros::ok){

        marker_pub.publish(points);
        if(robot_run){

           // cout << nova << endl;
            if(stara != nova){ //za pomik skozi vrsto
            //CMD_MSG.angular.z = nova;
                //cout << "stara != nova" << endl;
                CMD_MSG.angular.z = nova;
                CMD_MSG.linear.x = 0.2;
                premik.publish(CMD_MSG);

                cout << nova << endl;
/*
            if(nova < 0){
                //cout << "zSmer: " << zSmer << " Desno" << " yO: " << yOdmik  << " kL " << kaL << " kD " << kaD<< endl;
                cout << "zSmer: " << zSmer << " Desno, yOdmik: " << yOdmik<<endl;
                //cout << "Stevec: " << xOdmik/100  << " Desno" << endl;
            }else{
                //cout << "zSmer: " << zSmer << " Levo" << " yO: " << yOdmik << " kL " << kaL << " kD " << kaD << endl;
                cout << "zSmer: " << zSmer << " Levo, yOdmik:" << yOdmik  << endl;
                //cout << "Stevec: " << xOdmik/100  << " Levo, yOdmik:" <<  << endl;
            }*/
            stara = nova;
        }
    }


        nh.getParam("/farmbeast_auto", robot_run);
        nh.getParam("/farmbeast_task_select", farmbeast_task_select);
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
