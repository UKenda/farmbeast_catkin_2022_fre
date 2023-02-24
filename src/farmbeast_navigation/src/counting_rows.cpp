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
#include<unistd.h>


using namespace std;
using namespace geometry_msgs;

#define PI 3.14159265


//CMD_Vel
geometry_msgs::Twist CMD_MSG;

unsigned int microsecond = 1000000;
bool loop = true, zaznava = true, sprememba = false, first_loop = true;
int stevec_vrste = 0;
int izvajanje = 0; // 0 - ne deluje ta file (vozi naravnost), 1 -  desno, 2 - levo
int vmesna_tocka = 0; // poglej v while - za zacetek
int konec_obracanja = 0; 

int zeljeno_st_vrst [10] = {1,1,1,1,1,1,2,2,3,2};
bool nazaj_v_vrsto = false;
int stevilo_zavijanja = 0;
//CALLBACK

void LaserCB_desno(const sensor_msgs::LaserScan::ConstPtr& scan){

    if (izvajanje == 1)
    {
        int stevec = 0;
        for(int l = 0; l <= (scan->ranges.size()-1); l++) //pregledamo vse vrednosti iz lidarja
        {
            if(scan->ranges[l] < 2 && scan->ranges[l] > -2) //ce smo v kateri koli meritvi kaj zaznali povecaj stevec za 1
            {
                stevec++;
            }
        }
        if (stevec == 0) zaznava = false; //ce nismo nic zaznali smo na sredini vrste
        else zaznava = true;

        if (zaznava == false && sprememba == false)
        {
            sprememba = true;
            stevec_vrste++;
        }
        else if (zaznava != false) sprememba = false;
        
        cout << "Stevec_vrste " << stevec_vrste << endl;
        cout << "Zaznava " << zaznava << endl;
        cout << "Sprememba " << sprememba << endl;
        cout << "Stevec " << stevec << endl;
        cout << " "  << endl;

        if(zeljeno_st_vrst[stevilo_zavijanja] == stevec_vrste)
        {
            izvajanje = 0;
            nazaj_v_vrsto = true;
            stevilo_zavijanja++;
        }
        CMD_MSG.angular.z = 0;
        CMD_MSG.linear.x = 0.5;
        //loop=true;
    }
    else if (izvajanje == 0)
    {
        zaznava = true; 
        sprememba = false;
        stevec_vrste = 0;
    }
}

void LaserCB_levo(const sensor_msgs::LaserScan::ConstPtr& scan){

    if (izvajanje == 2)
    {
    cout << "izvajanje jea 2ka" << endl;
        int stevec = 0;
        for(int l = 0; l <= (scan->ranges.size()-1); l++) //pregledamo vse vrednosti iz lidarja
        {
            if(scan->ranges[l] < 2 && scan->ranges[l] > -2) //ce smo v kateri koli meritvi kaj zaznali povecaj stevec za 1
            {
                stevec++;
            }
        }
        if (stevec == 0) zaznava = false; //ce nismo nic zaznali smo na sredini vrste
        else zaznava = true;

        if (zaznava == false && sprememba == false)
        {
            sprememba = true;
            stevec_vrste++;
        }
        else if (zaznava != false) sprememba = false;
        
        cout << "Stevec_vrste " << stevec_vrste << endl;
        cout << "Zaznava " << zaznava << endl;
        cout << "Sprememba " << sprememba << endl;
        cout << "Stevec " << stevec << endl;
        cout << " "  << endl;

        if(zeljeno_st_vrst[stevilo_zavijanja] == stevec_vrste) //dosezemo zeljeno vrsto
        {
            izvajanje = 0;
            cout << "dosezena vrsta" << endl;
            nazaj_v_vrsto = true; //nadaljuje se v while if3.0
            stevilo_zavijanja++;
        }

        CMD_MSG.angular.z = 0;
        CMD_MSG.linear.x = 0.5;
    }
    else if (izvajanje == 0)
    {


        zaznava = true; 
        sprememba = false;
        stevec_vrste = 0;
    }
}



int main(int argc, char **argv){
    ros::init(argc, argv, "counting_row_node");
    ros::NodeHandle nh;
    ros::Rate r(10);
    cout << "zdravo!" << endl;
    nh.setParam("/farmbeast/counting_rows",true);
    //sub/pub
    //ros::Subscriber laser = nh.subscribe("/camera/scan",10, LaserCB);
    ros::Subscriber laser_levo = nh.subscribe("/camera/scan_levo",10, LaserCB_levo);
    ros::Subscriber laser_desno = nh.subscribe("/camera/scan_desno",10, LaserCB_desno);

    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    cout << "pred whille" << endl;
    nh.getParam("/farmbeast/counting_rows",loop);
    while(loop){
        nh.getParam("/farmbeast/izvajanje",vmesna_tocka);
        if (vmesna_tocka != 0 && first_loop == true) //caka na konec vrste
        {
            cout << "prvi if" << endl;
            first_loop = false;


                CMD_MSG.linear.x = 0.5;
                CMD_MSG.angular.z = 0.0;
                premik.publish(CMD_MSG);
                usleep(2.75*microsecond);//sleeps for 3 second

            cout << "after for" << endl;
            nh.setParam("/farmbeast/obracanje",vmesna_tocka);
            nh.getParam("/farmbeast/obracanje",konec_obracanja);
            while(konec_obracanja != 0)  //caka dokler ni obrnjen ua 90 stopnij
            {
                nh.getParam("/farmbeast/obracanje",konec_obracanja);
            }
            nh.setParam("/farmbeast/idle/tekmovanje",true);
            cout << "after while" << endl;
            izvajanje = vmesna_tocka; //da vemo katero stran gledali (levo ali desno)... nadaljuje se v callback-ih
        }
        else if (vmesna_tocka == 0)
        {
            //cout << "vmesna tocka = 0s" << endl;

            first_loop = true;
        }
        else{
            //cout << "else" << endl;

            premik.publish(CMD_MSG);
        }
        if (nazaj_v_vrsto == true) // if3.0
        {

                CMD_MSG.linear.x = 0.5;
                CMD_MSG.angular.z = 0.0;
                premik.publish(CMD_MSG);
                usleep(2*microsecond);//sleeps for 1 second
            
            nazaj_v_vrsto = false;
            nh.setParam("/farmbeast/obracanje",vmesna_tocka);
            nh.getParam("/farmbeast/obracanje",konec_obracanja);
            while(konec_obracanja != 0)  //caka dokler ni obrnjen za 90 stopnij
            {
                nh.getParam("/farmbeast/obracanje",konec_obracanja);
            }
            nh.setParam("/farmbeast/idle/tekmovanje",true);
            nh.setParam("/farmbeast/izvajanje",0);
        }
        //loop=false;
        
        ros::spinOnce();
        r.sleep();
        nh.getParam("/farmbeast/counting_rows",loop);


    }

    cout << "konec" << endl;

    return 0;
}