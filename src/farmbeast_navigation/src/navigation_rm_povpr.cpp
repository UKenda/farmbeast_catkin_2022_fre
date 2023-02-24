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
#include <fstream>
#include <time.h>
#include <sys/time.h>

using namespace std;
using namespace geometry_msgs;

#define PI 3.14159265

    //koliko stopinj bomo dejansko prebrali na vsaki strani
    int st_mer = 30;

    timeval zac; //ko pridemo na konec vrste
    timeval kon;  //se vedno spreminja


    float obrni = -1;
    int farmbeast_task_select=1;
    bool robot_run = false;

    //izracunani koti
    vector<float> cos_kot;
    float value;
    int i = 0;
    float napaka = 0.02;

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


    //stara/nova
    float stara = 0;
    float nova = 0;
    int obr_st = 0;
    int obr_no = 0;

    bool l1 = false; //če sta l1 in l2 true, potem smo prišli na konec vrste
    bool l2 = false;

    //CMD_Vel
    geometry_msgs::Twist CMD_MSG;


    //marker
    visualization_msgs::Marker points;
    geometry_msgs::Point p;



//CALLBACK
void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){

    points.DELETEALL;
    // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;





        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;



        //skozi vrsto

            //leva premica

            i = 0;

            for(int l = 45; l <= 45 + st_mer; l++){
                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                    kot_L.push_back(l - 45); //pretvorba kota za levo stran
                    povpL = povpL + mer_L[i]*cos_kot[kot_L[i]];
                    i++;
                }
            }

            xsumL = 0; x2sumL = 0; ysumL = 0; xysumL = 0;

            povpL = povpL/mer_L.size();


            //desna premica

            i = 0;

            for(int l = 225; l >= 225 - st_mer; l--){

                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_D.push_back(scan->ranges[l]);
                    kot_D.push_back(225 - l); //pretvorba kota za desno stran
                    povpD = povpD + mer_D[i]*cos_kot[kot_D[i]];
                    i++;
                }
            }

            xsumD = 0; x2sumD = 0; ysumD = 0; xysumD = 0;

            povpD = povpD/mer_D.size();


/*
            for(int i = 0; i < mer_D.size(); i++){
                xsumD = xsumD + mer_D[i];
                ysumD = ysumD + kot_D[i];
                x2sumD = x2sumD + pow(mer_D[i],2);
                xysumD = xysumD + mer_D[i]*kot_D[i];
            }

            kaD = (mer_D.size()*xysumD-xsumD*ysumD)/(mer_D.size()*x2sumD-xsumD*xsumD); //koeficient
            naD = (x2sumD*ysumD-xsumD*xysumD)/(x2sumD*mer_D.size()-xsumD*xsumD);*/

           // xOdmik = ((naD - naL)/(kaL - kaD)); //x koordinata
            //cout << xOdmik << endl;
           /* yOdmik = kaL*xOdmik + naL; //y koordinata

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
                        zSmer = 0.3;*/
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
           /* }else{
                    if(zSmer > -0.05)
                        zSmer = 0;
                    else if(zSmer > -0.1)
                        zSmer = zSmer;
                    else if(zSmer > -0.15)
                        zSmer = -0.15;
                    else if(zSmer > -0.2)
                        zSmer = -0.2;
                    else
                        zSmer = -0.3;*/
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
            //}

           /* zSmer = zSmer*max_obr;



            nova = obrni*zSmer;*/
            //cout << nova << " " << zSmer << endl;


            //cout << "PovpL: " << fabs(povpL) << " " << mer_L.size() <<  "  || PovpD: " << fabs(povpD) << " " << mer_D.size() << endl;

            if(fabs(povpL) >= (fabs(povpD) - napaka) && fabs(povpL) <= (fabs(povpD) + napaka)){ //če smo na sredini
                //cout << "NA SREDINI SMO!!! " << povpL << " " << povpD << endl;
                obr_no = 0;
            }else{
                if(fabs(povpL) > fabs(povpD)){
                    //cout << "Pojdi proti levi! " << povpL << " " << povpD << endl;
                    nova = 0.1;
                    obr_no = 1;

                }else{
                    //cout << "Pojdi proti desni! " << povpL << " " << povpD << endl;
                    nova = -0.1;
                    obr_no = 2;
                }

            }


            if(obr_st != obr_no){
                switch(obr_no){
                    case 0:{
                        cout << "NA SREDINI SMO!!! " << povpL << " " << povpD << endl;
                        break;
                    }
                    case 1:{
                        cout << "Pojdi proti levi! " << povpL << " " << povpD << endl;
                        break;
                    }
                    case 2:{
                        cout << "Pojdi proti desni! " << povpL << " " << povpD << endl;
                        break;
                    }


                }
            }


            povpL = 0;
            povpD = 0;

            obr_st = obr_no;

            mer_L.clear();
            mer_D.clear();
            kot_L.clear();
            kot_D.clear();
            //gettimeofday(&kon, NULL);
           // cout << kon.tv_usec - zac.tv_usec << endl;
    //}
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    cout << "zdravo!" << endl;

    if(nh.hasParam("/navigation_task1/obrni") && nh.hasParam("/navigation_task1/min_val") && nh.hasParam("/navigation_task1/max_val")){

        nh.getParam("/navigation_task1/min_val", min_val);
        nh.getParam("/navigation_task1/max_val", max_val);
        nh.getParam("/navigation_task1/obrni", obrni);
        nh.getParam("/farmbeast_auto", robot_run);
        nh.getParam("/farmbeast_task_select", farmbeast_task_select);
        nh.getParam("/navigation_task1/max_obr", max_obr);
        nh.getParam("/navigation_task1/xSmer", xSmer);
        nh.getParam("/navigation_task1/st_mer", st_mer);
    }else{
        cout << "Ni naslo parametra" << endl;
    }
    ifstream myFile;
    myFile.open("kot_cos.txt");
    if (myFile.is_open())
        {
            cout << "notri smo" << endl;
            while(myFile>>value){
                myFile>>value;
                cout << "Prebrali smo nekaj " << value << endl;
                cos_kot.push_back(value);
            }

            myFile.close();
        }else{
        cout << "Ni datoteke" << endl;
        ofstream myFileOut;
        myFileOut.open("kot_cos.txt", ios::app);
        if(myFileOut.is_open()){
            for(int kot = 0; kot < 270; kot++){
                value = cos((kot*PI)/180);
                myFileOut << kot << " " << value << endl;
                cout << kot << " " << value << endl;
                cos_kot.push_back(value);

            }
            myFileOut.close();
        }
    }

    cout << "Velikost vektorja " << cos_kot.size() << endl;

    //sub/pub
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Subscriber laser = nh.subscribe("/scan",10, LaserCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);


    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;

    while(ros::ok){
       // gettimeofday(&zac, NULL);
        marker_pub.publish(points);
        //robot_run


        if(stara != nova){ //za pomik skozi vrsto
        //CMD_MSG.angular.z = nova;
            //cout << "stara != nova" << endl;
            CMD_MSG.angular.z = nova;
            CMD_MSG.linear.x = 0.1;
            //premik.publish(CMD_MSG);

           cout << nova << endl;

        stara = nova;
    }


        /*if(robot_run){


            if(stara != nova){ //za pomik skozi vrsto
            //CMD_MSG.angular.z = nova;
                //cout << "stara != nova" << endl;
                CMD_MSG.angular.z = nova;
                CMD_MSG.linear.x = 0.1;
                //premik.publish(CMD_MSG);

               // cout << nova << endl;

            stara = nova;
        }
    }*/
        ros::spinOnce();
    }


    return 0;
}
