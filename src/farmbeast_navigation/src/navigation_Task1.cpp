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
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <string>

using namespace std;
using namespace geometry_msgs;

//

    //merjenje časa
    timeval zac; //ko pridemo na konec vrste
    timeval kon;  //se vedno spreminja
    float pre_pot = 0; //prepotovana pot
    float pospesek;

    //branje datoteke
    int value;
    vector<int> cos_kot;
    int prStevec = 0; //z njim se pomikamo skozi vektor
    int prStopnja = 0; //zavijanje

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
    float xSmer;
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
    float max_val = 0.60;
    float xOdmik;
    float yOdmik;
    float yOdmik_sredina = -145; //treba vnesti, ko stestiramo
    float yMax = 200; //največja razlika med koefientoma, TREBA NASTAVITI

    //stetje vrst
    int sStevec = 0; //štetje vrst
    int sMinS; //minimalni .size() vektorja, ki ga moremo dosečti, da je vrsta zaznan

    //stara/nova
    float stara = 0;
    float nova;

    bool l1; //če sta l1 in l2 true, potem smo prišli na konec vrste
    bool l2;

    bool robot_run = false;
    int farmbeast_task_select=1;

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

    points.DELETEALL;
    // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;





        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;



    if(prStevec == 0 && cos_kot[prStevec] == 0){
        CMD_MSG.linear.x = 0.2;
        CMD_MSG.angular.z = 0;
        prStevec++;
        l1 = true;
        l2 = true;
    }else if(l1 && l2){
        if(cos_kot[prStevec] == 1){//prišli smo do konca vrste in se moramo obrniti proti desno
            if(prStopnja == 0){
                cout << "Stopnja " << prStopnja << endl;
                CMD_MSG.linear.x = 0;
                if(oTrenutna > oCenter + oMeja){
                    CMD_MSG.angular.z = -0.1; //se pomaknemo v desno
                }else if(oTrenutna < oCenter - oMeja){
                    CMD_MSG.angular.z = 0.1;
                }else{
                    prStopnja++;
                    CMD_MSG.angular.z = 0;
                    CMD_MSG.linear.x = 0.1;
                    gettimeofday(&zac, NULL);
                }

                //if(CMD_MSG.linear.x = 0.1);
            }else if(prStopnja == 1){
                gettimeofday(&kon, NULL);
                pre_pot = pre_pot + (pospesek*(float)pow((float)(kon.tv_usec - zac.tv_usec)/1000000 + (float)(kon.tv_sec - zac.tv_sec),2))/2;
                if(prStopnja < 0.7)
                    prStopnja--;
                else
                    prStopnja++;
                CMD_MSG.linear.x = 0.1;
            }else if(prStopnja == 2){
                cout << "Stopnja " << prStopnja << endl;
                CMD_MSG.linear.x = 0;
                if(oTrenutna > oCenter - oMax)
                    CMD_MSG.angular.z = -0.2;
                else{
                    CMD_MSG.angular.z = 0;
                    prStopnja++;
                }
            }else if(prStopnja == 3){
                cout << "Stopnja " << prStopnja << endl;
                CMD_MSG.linear.x = 0.1;
            }else if(prStopnja == 4){
                cout << "Stopnja " << prStopnja << endl;
                CMD_MSG.linear.x = 0;
                if(oTrenutna > oCenter - 2*oMax)
                    CMD_MSG.angular.z = -0.2;
                else{
                    CMD_MSG.angular.z = 0;
                    prStopnja++;
                }
            }
        }else if(cos_kot[prStevec] == -1){
            if(prStopnja == 0){
                cout << "Stopnja " << prStopnja << endl;
                CMD_MSG.linear.x = 0;
                if(oTrenutna > oCenter + oMeja){
                    CMD_MSG.angular.z = -0.1; //se pomaknemo v desno
                }else if(oTrenutna < oCenter - oMeja){
                    CMD_MSG.angular.z = 0.1;
                }else{
                    prStopnja++;
                    CMD_MSG.angular.z = 0;
                    CMD_MSG.linear.x = 0.1;
                    gettimeofday(&zac, NULL);
                }

                //if(CMD_MSG.linear.x = 0.1);
            }else if(prStopnja == 1){
                gettimeofday(&kon, NULL);
                pre_pot = pre_pot + (pospesek*(float)pow((float)(kon.tv_usec - zac.tv_usec)/1000000 + (float)(kon.tv_sec - zac.tv_sec),2))/2;
                if(prStopnja < 0.7)
                    prStopnja--;
                else
                    prStopnja++;
                CMD_MSG.linear.x = 0.1;
            }else if(prStopnja == 2){
                CMD_MSG.linear.x = 0;
                if(oTrenutna < oCenter + oMax)
                    CMD_MSG.angular.z = 0.2;
                else{
                    CMD_MSG.angular.z = 0;
                    prStopnja++;
                }
            }else if(prStopnja == 3){
                CMD_MSG.linear.x = 0.1;
            }else if(prStopnja == 4){
                CMD_MSG.linear.x = 0;
                if(oTrenutna <= oCenter + 2*oMax)
                    CMD_MSG.angular.z = 0.2;
                else{
                    CMD_MSG.angular.z = 0;
                    prStopnja++;
                }
            }
        }else if(cos_kot[prStevec] == 0){
            //USTAVI SE
            CMD_MSG.linear.x = 0;
            CMD_MSG.angular.z = 0;
        }
    }else{
        //skozi vrsto

            //leva premica

            for(int l = 45; l <= 75; l++){
                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                    kot_L.push_back(l - 45); //pretvorba kota za levo stran
                }
            }

            if(mer_L.size() < 3){
                mer_L.clear();
                kot_L.clear();
                for(int l = 75; l <= 105; l++){
                    if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                        mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                        kot_L.push_back(l - 45); //pretvorba kota za levo stran
                    }
                }
                if(mer_L.size() < 3){
                    mer_L.clear();
                    kot_L.clear();
                    for(int l = 105; l <= 130; l++){
                        if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                            mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                            kot_L.push_back(l - 45); //pretvorba kota za levo stran
                        }
                    }
                    if(mer_L.size() < 3){
                        l1 = true;
                    }
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

            if(mer_D.size() < 3){
                mer_D.clear();
                kot_D.clear();
                for(int l = 195; l >= 165; l--){

                    if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                        mer_D.push_back(scan->ranges[l]);
                        kot_D.push_back(225 - l); //pretvorba kota za desno stran
                    }
                }
                if(mer_D.size() < 3){
                    mer_D.clear();
                    kot_D.clear();
                    for(int l = 165; l >= 140; l--){

                        if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                            mer_D.push_back(scan->ranges[l]);
                            kot_D.push_back(225 - l); //pretvorba kota za desno stran
                        }
                    }
                    if(mer_D.size() <3){
                        l2 = true;
                    }
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
                    else
                        zSmer = 0.3;
                    /*else if(zSmer < 0.4)
                        zSmer = 0.4;
                    else if(zSmer < 0.5)
                        zSmer = 0.5;
                    else if(zSmer < 0.7)
                        zSmer = 0.7;
                    else if(zSmer < 0.9)
                        zSmer = 0.9;
                    else
                        zSmer = 1;*/
            }else{
                    if(zSmer > -0.05)
                        zSmer = 0;
                    else if(zSmer > -0.1)
                        zSmer = zSmer;
                    else if(zSmer > -0.15)
                        zSmer = -0.15;
                    else if(zSmer > -0.2)
                        zSmer = -0.2;
                    else
                        zSmer = -0.3;
                    /*else if(zSmer > -0.4)
                        zSmer = -0.4;
                    else if(zSmer > -0.5)
                        zSmer = -0.5;
                    else if(zSmer > -0.7)
                        zSmer = -0.7;
                    else if(zSmer > -0.9)
                        zSmer = -0.9;
                    else
                        zSmer = -1;*/
            }

            if((yOdmik < yOdmik_sredina + 10.0) && (yOdmik > yOdmik_sredina - 10.0)){
                    zSmer = ((abs(kaL)-abs(kaD))*0.6)/yMax;
                    yPremik = true;
                    //cout << "Tu smo" << endl;
            }

            nova = zSmer;

            mer_L.clear();
            mer_D.clear();
            kot_L.clear();
            kot_D.clear();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    //branje
    std::ifstream myFile;
    myFile.open("/etc/farmbeast_txt/task1nav.txt", std::ios::app);
    if (myFile.is_open())
        {
            cout << "notri smo" << endl;
            while(myFile>>value){
                //myFile >> value;
                cout << "abc" << endl;
                cos_kot.push_back(value);
            }


            myFile.close();
            cout << "Smo zapisali... " << cos_kot.size() << endl;
        }else
            cout << "error" << endl;

    //sub/pub
    ros::Subscriber compas = nh.subscribe("/imu/data", 10, CompasCB);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber laser = nh.subscribe("/scan",10, LaserCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);



    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;

    while(ros::ok){

        if(robot_run == true)
        {
        //cout << "Smo not" << endl;
        marker_pub.publish(points);
        if(l1 && l2){
           //premik.publish(CMD_MSG);
           if(cos_kot[prStevec-1] == 0){
               premik.publish(CMD_MSG);
               sleep(1);
               l1 = false;
               l2 = false;
           }else if(cos_kot[prStevec] == 1){
               if(prStopnja == 1 || prStopnja == 0){
                   /*
                   if(CMD_MSG.linear.x == 0.1){
                       gettimeofday(&zac, NULL);

                   }else*/
                   premik.publish(CMD_MSG);
               }else if(prStopnja == 2){
                   premik.publish(CMD_MSG);
               }else if(prStopnja == 3){
                   premik.publish(CMD_MSG);
                   sleep(2);
                   //prStopnja++;
               }else if(prStopnja == 4)
                   premik.publish(CMD_MSG);
               else if(prStopnja == 5){
                   premik.publish(CMD_MSG);
                   usleep(500);
                   prStopnja = 0;
                   l1 = false;
                   l2 = false;
                   prStevec++;
                   CMD_MSG.linear.x = 0.2;
                   premik.publish(CMD_MSG);
               }
           }else if(cos_kot[prStevec] == -1){
               if(prStopnja == 1 || prStopnja == 0){
                   /*
                   if(CMD_MSG.linear.x == 0.1){
                       gettimeofday(&zac, NULL);

                   }else*/
                   premik.publish(CMD_MSG);
               }else if(prStopnja == 2){
                   premik.publish(CMD_MSG);
               }else if(prStopnja == 3){
                   premik.publish(CMD_MSG);
                   sleep(2);
                   prStopnja++;
               }else if(prStopnja == 4)
                   premik.publish(CMD_MSG);
               else if(prStopnja == 5){
                   premik.publish(CMD_MSG);
                   usleep(500);
                   prStopnja = 0;
                   l1 = false;
                   l2 = false;
                   prStevec++;
                   CMD_MSG.linear.x = 0.2;
                   premik.publish(CMD_MSG);
               }else if(cos_kot[prStevec] == 0){
                   premik.publish(CMD_MSG);
                   return 0;
               }
           }
        }else if(stara != nova){ //za pomik skozi vrsto
            CMD_MSG.angular.z = nova;
            //CMD_MSG.angular.z = 0;

            premik.publish(CMD_MSG);

            if(zSmer < 0){
                //cout << "zSmer: " << zSmer << " Desno" << " yO: " << yOdmik  << " kL " << kaL << " kD " << kaD<< endl;
                cout << "zSmer: " << zSmer << " Desno" << " yO: " << yOdmik  << " razlika: " << abs(kaL) - abs(kaD) << endl;
                //cout << "Stevec: " << xOdmik/100  << " Desno" << endl;
            }else{
                //cout << "zSmer: " << zSmer << " Levo" << " yO: " << yOdmik << " kL " << kaL << " kD " << kaD << endl;
                cout << "zSmer: " << zSmer << " Levo" << " yO: " << yOdmik  << " razlika: " << abs(kaL) - abs(kaD) << endl;
                //cout << "Stevec: " << xOdmik/100  << " Levo" << endl;
            }

        }
/*
        if(zSmer < 0){
            //cout << "zSmer: " << zSmer << " Desno" << " yO: " << yOdmik  << " kL " << kaL << " kD " << kaD<< endl;
            cout << "zSmer: " << zSmer << " Desno" << " yO: " << yOdmik  << " razlika: " << abs(kaL) - abs(kaD) << endl;
            //cout << "Stevec: " << xOdmik/100  << " Desno" << endl;
        }else{
            //cout << "zSmer: " << zSmer << " Levo" << " yO: " << yOdmik << " kL " << kaL << " kD " << kaD << endl;
            cout << "zSmer: " << zSmer << " Levo" << " yO: " << yOdmik  << " razlika: " << abs(kaL) - abs(kaD) << endl;
            //cout << "Stevec: " << xOdmik/100  << " Levo" << endl;


        }
*/
        }

        nh.getParam("farmbeast_auto", robot_run);
        nh.getParam("farmbeast_task_select", farmbeast_task_select);

        ros::spinOnce();
    }


    return 0;
}
