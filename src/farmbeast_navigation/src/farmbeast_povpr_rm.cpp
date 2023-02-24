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

int voznja_nacin;

//CMD_Vel
geometry_msgs::Twist CMD_MSG;


//marker
visualization_msgs::Marker points;
geometry_msgs::Point p;


bool ZACNI = true;
//CALLBACK
void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){

    points.DELETEALL;
    // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;





        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;
   if(robot_run){
        if(voznja_nacin == 0){

        //skozi vrsto

            //leva premica

            for(int l = 60; l <= 60 + st_mer; l++){
                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                    kot_L.push_back(l - 45); //pretvorba kota za levo stran
                }
            }

            xsumL = 0; x2sumL = 0; ysumL = 0; xysumL = 0;

            if(mer_L.size() <2){
                for(int l = 75; l <= 75 + st_mer && l <= 134; l++){
                    if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                        mer_L.push_back((scan->ranges[l])*(-1)); //vrednosti morajo biti negativne, saj bi bile tudi na koordinatnem sistemu
                        kot_L.push_back(l - 45); //pretvorba kota za levo stran
                    }
                }

                if(mer_L.size() <2){
                    leva_niT = true;
                }
            }else{
                leva_niT = false;
                for(int i = 0; i < mer_L.size(); i++){
                    povpL = povpL + mer_L[i]*cos_kot[kot_L[i]];
                }

                povpL = povpL/mer_L.size();
                absPovpL = fabs(povpL);
            }






/*
            for(int i = 0; i < mer_L.size(); i++){
                xsumL = xsumL + mer_L[i];
                ysumL = ysumL + kot_L[i];
                x2sumL = x2sumL + pow(mer_L[i],2);
                xysumL = xysumL + mer_L[i]*kot_L[i];
            }

           kaL = (mer_L.size()*xysumL-xsumL*ysumL)/(mer_L.size()*x2sumL-xsumL*xsumL); //koeficient
            naL = (x2sumL*ysumL-xsumL*xysumL)/(x2sumL*mer_L.size()-xsumL*xsumL);
*/


            //desna premica
            for(int l = 210; l >= 210 - st_mer; l--){

                if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                    mer_D.push_back(scan->ranges[l]);
                    kot_D.push_back(225 - l); //pretvorba kota za desno stran
                }
            }

            xsumD = 0; x2sumD = 0; ysumD = 0; xysumD = 0;

            /*if(mer_D.size() == 0 && mer_L.size() == 0){
                ustavi = true;
                return;
            }*/

            if(mer_D.size() <2){
                for(int l = 195; l >= 195 - st_mer && l >= 136; l--){

                    if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val)){
                        mer_D.push_back(scan->ranges[l]);
                        kot_D.push_back(225 - l); //pretvorba kota za desno stran
                    }
                }

                if(mer_D.size() < 2 ){
                    desna_niT = true;
                }
            }else{
                desna_niT = false;
                //povprecje
                for(int i = 0; i < mer_D.size(); i++){
                    povpD = povpD + mer_D[i]*cos_kot[kot_D[i]];
                }

                povpD = povpD/mer_D.size();
                absPovpD = fabs(povpD);
            }






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


            if(mer_D.size() == 0 && mer_L.size() == 0){
                            ustavi = true;
                            cout << "ne najdem nic" << endl;
			    leva_niT = false;
			    desna_niT = false;
                            return;
                        }


            //cout << mer_D.size() << " " << mer_L.size() << " ";

            if(!leva_niT && !desna_niT){ //imamo obe strani
               
                    //cout << "PovpL: " << povpL << " || PovpD: " << povpD << endl;

                    nova = ((absPovpL - absPovpD)/(absPovpL + absPovpD))*koeficient;

                    if(nova <= 0.9 && nova >= -0.9){
                         //cout << nova << endl;
                    }else{
                        if(nova  >= 0.9)
                            nova = 0.9;
                        else
                            nova = -0.9;
                    }

                    if(absPovpL > absPovpD){
                        cout << "Pojdi proti levi! " << nova << " " <<  absPovpL << " " << absPovpD << endl;

                    }else{
                        cout << "Pojdi proti desni! " << nova << " " << absPovpL << " " << absPovpD << endl;

                    }

                    povpL = 0;
                    povpD = 0;

                    mer_L.clear();
                    mer_D.clear();
                    kot_L.clear();
                    kot_D.clear();
                }
            }else if(leva_niT && !desna_niT){ //imamo samo desno stran
                float inv_WL = 0.75 - absPovpD;
                cout << inv_WL << " " << absPovpD << endl;
                if(inv_WL >= 0){
                    nova = ((inv_WL - absPovpD)/(inv_WL + absPovpD))*koeficient;
                    if(nova <= 0.9 && nova >= -0.9){
                         //cout << nova << endl;
                    }else{
                        if(nova  >= 0.9)
                            nova = 0.9;
                        else
                            nova = -0.9;
                    }

                }else{
                    nova = -0.9;
                }
            }else if(!leva_niT && desna_niT){ //imamo samo levo stran
                float inv_WD = 0.75 - absPovpL;
                cout << inv_WD << " " << absPovpL << endl;
                if(inv_WD >= 0){
                    nova = ((absPovpL - inv_WD)/(inv_WD + absPovpL))*koeficient;
                    if(nova <= 0.9 && nova >= -0.9){
                         //cout << nova << endl;
                    }else{
                        if(nova  >= 0.9)
                            nova = 0.9;
                        else
                            nova = -0.9;
                    }

                }else{
                    nova = 0.9;
                }
            }


            /*if(mer_D.size() == 0 && mer_L.size() == 0){
                ustavi = true;
                cout << "ne najdem nic" << endl;
                return;
            }*/

            povpL = 0;
            povpD = 0;

            mer_L.clear();
            mer_D.clear();
            kot_L.clear();
            kot_D.clear();

	    leva_niT = false;
	    desna_niT = false;

 	if(nova >= 0){
                if(nova <= 0.05)
                        nova = nova;
                else if(nova < 0.1)
                        nova = 0.1;
                else if(nova < 0.15)
                        nova = 0.15;
                else if(nova < 0.2)
                        nova = 0.2;
                else if(nova < 0.3)
                        nova = 0.3;
                else if(nova < 0.4)
                        nova = 0.4;
                else if(nova < 0.5)
                        nova = 0.5;
                else if(nova < 0.6)
                        nova = 0.6;
                else if(nova < 0.7)
                        nova = 0.7;
                else if(nova < 0.8)
                        nova = 0.8;
                else if(nova < 0.9)
                        nova = 0.9;
                else
                        nova = 0.9;
        }else if(nova < 0){
                if(nova >= -0.05)
                        nova = nova;
                else if(nova > -0.1)
                        nova = -0.1;
                else if(nova > -0.15)
                        nova = -0.15;
                else if(nova > -0.2)
                        nova = -0.2;
                else if(nova > -0.3)
                        nova = -0.3;
                else if(nova > -0.4)
                        nova = -0.4;
                else if(nova > -0.5)
                        nova = -0.5;
                else if(nova > -0.6)
                        nova = -0.6;
                else if(nova > -0.7)
                        nova = -0.7;
                else if(nova > -0.8) 
                        nova = -0.8;
                else if(nova > -0.9)
                        nova = -0.9;
                else
                        nova = -0.9;

        }

    
   }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    cout << "zdravo!" << endl;

	bool nov_zac = true;
	nh.setParam("drive_mode_skeed",false);
	nh.setParam("drive_mode_ackerman",true);
    nh.setParam("/navigation_task1/kot_cal", 1);

    if(nh.hasParam("/navigation_task1/obrni") && nh.hasParam("/navigation_task1/min_val") && nh.hasParam("/navigation_task1/max_val")){

        nh.getParam("/navigation_task1/min_val", min_val);
        nh.getParam("/navigation_task1/max_val", max_val);
        nh.getParam("/navigation_task1/obrni", obrni);
        nh.getParam("/farmbeast_auto", robot_run);
        nh.getParam("/farmbeast_task_select", farmbeast_task_select);
        nh.getParam("/navigation_task1/max_obr", max_obr);
        nh.getParam("/navigation_task1/xSmer", xSmer);
        nh.getParam("/navigation_task1/st_mer", st_mer);
        nh.getParam("/navigation_task1/hitrost", hitrost);
        nh.getParam("/navigation_task1/koef", koeficient);
        nh.getParam("/navigation_task1/ack_nac", ack_nac);
        nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);

    }else{
        cout << "Ni naslo parametra" << endl;
    }


    nh.setParam("ackerman_drive_mode",ack_nac);

    koeficient = hitrost*koeficient;

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

    double rate = 10.0;
    ros::Rate r(rate);



    while(ros::ok){

	

        marker_pub.publish(points);

        nh.getParam("/farmbeast_auto", robot_run);
       	if(robot_run){


                if(voznja_nacin == 0){

                if(nov_zac){
                nh.setParam("ackerman_drive_mode", ack_nac);
                nov_zac = false;
                }


                    if(stara != nova){ //za pomik skozi vrsto
                    //CMD_MSG.angular.z = nova;
                        //cout << "stara != nova" << endl;
                        CMD_MSG.angular.z = nova;
			//CMD_MSG.linear.x = 0.0; //NE POZABI NAZAJ NASTIMAT!!
                        CMD_MSG.linear.x = hitrost;
                        premik.publish(CMD_MSG);

                       // cout << nova << endl;

                    stara = nova;
                    }


                 /*   if(robot_run){
                        cout << "tu smo" << endl;

                        if(stara != nova){ //za pomik skozi vrsto
                        //CMD_MSG.angular.z = nova;
                            cout << "stara != nova" << endl;
                            CMD_MSG.angular.z = nova;
                            CMD_MSG.linear.x = hitrost;
                            //premik.publish(CMD_MSG);

                           // cout << nova << endl;

                        stara = nova;
                    }
                }*/

                    if(ustavi){
               // nh.setParam("/farmbeast_base_driver/odo_reset",1);
                        CMD_MSG.angular.z = 0;
                        CMD_MSG.linear.x = 0;
                        premik.publish(CMD_MSG);
                        ustavi = false;
                        nh.setParam("/PhidgetsImuNodelet/nac_voznje", 1);
                        nov_zac = true;
                    }

                }
                nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);
        }
        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
