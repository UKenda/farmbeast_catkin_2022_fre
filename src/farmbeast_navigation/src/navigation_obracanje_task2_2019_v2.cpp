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
#include <time.h>
#include <sys/time.h>
#include <cmath>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace geometry_msgs;


    /*//merjenje časa
    timeval zac; //ko pridemo na konec vrste
    timeval kon;  //se vedno spreminja
    float pre_pot = 0; //prepotovana pot
    float pospesek;
    float pospesek0; //koliko meri pospešek, če smo na miru

    //IMU
    float oCenter = -0.2; //tej vrednosti se naj približujemo
    float oMeja = 0.05; //meja natančnosti
    float oMax = 0.57; //ko smo obrnjeni za 90 stopinj
    float o180 = oMax*2; //centru pristejemo/odstejemo to vrednost ko pridemo v novo vrsto
    float oTrenutna;*/

    bool calibrate_kot = true;
    bool poravnava = false;
    bool poravnava_zac = false;
    bool poravnava_kon = false;
    bool arewethereyet = false;
    int star_nacin = 100;
    bool poprvi = false;
    //IMU
    int smer;
    float o90 = 0.7;
    float o90Kon = 0.0;
    bool zacetek = true;
    bool konec = false;
    float oCenter = 0.0;
    float oTren = 0.0;
    float oPred = 0.0;
    float oRazl = 0.0;
    bool step1 = false;
    bool step2 = false;
    bool napr_zacetek = true;
    bool napr_konec = false;
    bool konec_zac = true;
    bool konec_kon = false;
    bool robot_run = false;

    bool zaznaliDesno = false;
      bool zaznaliLevo = false;
    vector<int> branje;

    bool stran_zacetek = true;
    bool stran_konec = false;

    int stevec = 0;
    int stevec1 = 0;

    float stara = 0.0;

    float osx = 0;
    float osy = 0;
    float osz = 0;
    float w = 0;

    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    double stopinje;
    double konc_stopinje;

    bool daj_na_ack = false;

   bool gremoNaprej = true;
   bool gremoStran = false;
   bool obrni = false;
    
   int stevec_kolkic = 0;
    //CMD_Vel
    geometry_msgs::Twist CMD_MSG;
    float zSmer = 0.0;
    float xHitrost;
    float zHitrost;
    int pot1 = 1000;
    int pot2 = 1000;
    int pot3 = 1000;
    float centriran_kot = 0.0;
    int napr_razd = 0;
    int stran_razd = 0;
    int voznja_nacin;
    int kot_cal = 0;
    int wheel1_pos = 0;
    int wheel2_pos = 0;
    int wheel3_pos = 0;
    int wheel4_pos = 0;
    int wheel1_tren = 0;
    int wheel2_tren = 0;
    int wheel3_tren = 0;
    int wheel4_tren = 0;
    int povprecje_pos, povprecje_tren;
    int st_vrst = 0;

void CompasCB(const sensor_msgs::Imu::ConstPtr& compas){
    //cout << "Orientacija x-osi: " << compas->orientation.x << endl;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "laser");

    ros::NodeHandle nh;

int value;
    ifstream myFile;
    myFile.open("/home/pi/PKP2017/src/Farmbeast/farmbeast_navigation/launch/task2_nav.txt");
  

    ros::Subscriber compas = nh.subscribe("/imu/data", 50, CompasCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    /*nh.setParam("drive_mode_skeed",false);
    nh.setParam("drive_mode_ackerman",true);*/
    int nazaj_razd = 0;
    if(nh.hasParam("/PhidgetsImuNodelet/smer")){
        /*
    <param name="casNaprej" value="1000000"/>
    <param name="casStran" value="1000000"/>
    <param name="hitrost" value="0.3"/>
    <param name="hitrost_z" value = "0.2"/>

         */
	napr_zacetek = true;

        nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);
        nh.getParam("/farmbeast_base_driver/wheel1_pos", wheel1_tren);
        nh.getParam("/farmbeast_base_driver/wheel2_pos", wheel2_tren);
        nh.getParam("/farmbeast_base_driver/wheel3_pos", wheel3_tren);
        nh.getParam("/farmbeast_base_driver/wheel4_pos", wheel4_tren);
	nh.getParam("/farmbeast_auto", robot_run);

    }else{
        cout << "Ni naslo parametra" << endl;
    }
        cout << "Prisli smo cez branje parametrov" << endl;

     //nh.setParam("ackerman_drive_mode",7);
	//smer = 1; 1 = desno, 0 = levo
    double rate = 10.0;
    ros::Rate r(rate);

	poravnava = false;
	gremoNaprej = true;
	napr_zacetek = true;
    //gremoNaprej = true;
    cout << "Smo pred zanko" << endl;
	int i = 0; //stetje vrst
    while(ros::ok){

	nh.getParam("/farmbeast_auto", robot_run);
 
	if(robot_run){
	//cout <<"V robot_run"<<endl;
		if(voznja_nacin == 0){
			//branje iz tekstovne datoteke
			nh.getParam("/farmbeast_base_driver/wheel1_pos",wheel1_tren);
			nh.getParam("/farmbeast_base_driver/wheel2_pos",wheel2_tren);
			nh.getParam("/farmbeast_base_driver/wheel3_pos",wheel3_tren);
			nh.getParam("/farmbeast_base_driver/wheel4_pos",wheel4_tren);
			if(napr_zacetek){
				povprecje_pos = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
		            	//povprecje_tren = povprecje_pos;
		            	napr_zacetek = false;
				//napr_razd = ; //TU SE MORAS BRAT IZ DATOTEKE PA POMNOZIT
				
			}
			
			povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
			//cout << povprecje_pos << " " << povprecje_tren << endl;
			ROS_INFO("%d! %d... %d? %d vrst!!!", povprecje_pos , povprecje_tren,povprecje_tren + st_vrst*napr_razd, napr_razd , st_vrst);
			if((povprecje_tren <= povprecje_pos + pot1)){
			    CMD_MSG.linear.x = 0.3;
			    CMD_MSG.angular.z = 0.0;
			    premik.publish(CMD_MSG);
			}else{
				CMD_MSG.linear.x = 0.0;
				CMD_MSG.angular.z = 0.0;
				premik.publish(CMD_MSG);
				usleep(1000000);

				voznja_nacin = 4;
				
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 4);//gremo naprej za x vrst
				napr_zacetek = true;
				//arewethereyet = false;
			}



		}else if(voznja_nacin == 4){
			//branje iz tekstovne datoteke
			nh.getParam("/farmbeast_base_driver/wheel1_pos",wheel1_tren);
			nh.getParam("/farmbeast_base_driver/wheel2_pos",wheel2_tren);
			nh.getParam("/farmbeast_base_driver/wheel3_pos",wheel3_tren);
			nh.getParam("/farmbeast_base_driver/wheel4_pos",wheel4_tren);
			if(napr_zacetek){
				povprecje_pos = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
		            	//povprecje_tren = povprecje_pos;
		            	napr_zacetek = false;
				//napr_razd = ; //TU SE MORAS BRAT IZ DATOTEKE PA POMNOZIT
				
			}
			
			povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
			//cout << povprecje_pos << " " << povprecje_tren << endl;
			ROS_INFO("%d! %d... %d? %d vrst!!!", povprecje_pos , povprecje_tren,povprecje_tren + st_vrst*napr_razd, napr_razd , st_vrst);
			if((povprecje_tren <= povprecje_pos + pot2)){
			    CMD_MSG.linear.x = 0.3;
			    CMD_MSG.angular.z = 0.0;
			    premik.publish(CMD_MSG);
			}else{
				CMD_MSG.linear.x = 0.0;
				CMD_MSG.angular.z = 0.0;
				premik.publish(CMD_MSG);
				usleep(1000000);

				voznja_nacin = 5;
				
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 5);//gremo naprej za x vrst
				napr_zacetek = true;
				//arewethereyet = false;
			}



		}else if(voznja_nacin == 5){
			//branje iz tekstovne datoteke
			nh.getParam("/farmbeast_base_driver/wheel1_pos",wheel1_tren);
			nh.getParam("/farmbeast_base_driver/wheel2_pos",wheel2_tren);
			nh.getParam("/farmbeast_base_driver/wheel3_pos",wheel3_tren);
			nh.getParam("/farmbeast_base_driver/wheel4_pos",wheel4_tren);
			if(napr_zacetek){
				povprecje_pos = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
		            	//povprecje_tren = povprecje_pos;
		            	napr_zacetek = false;
				//napr_razd = ; //TU SE MORAS BRAT IZ DATOTEKE PA POMNOZIT
				
			}
			
			povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
			//cout << povprecje_pos << " " << povprecje_tren << endl;
			ROS_INFO("%d! %d... %d? %d vrst!!!", povprecje_pos , povprecje_tren,povprecje_tren + st_vrst*napr_razd, napr_razd , st_vrst);
			if((povprecje_tren <= povprecje_pos + pot3)){
			    CMD_MSG.linear.x = 0.3;
			    CMD_MSG.angular.z = 0.0;
			    premik.publish(CMD_MSG);
			}else{
				CMD_MSG.linear.x = 0.0;
				CMD_MSG.angular.z = 0.0;
				premik.publish(CMD_MSG);
				usleep(1000000);

				voznja_nacin = 6;
				
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 6);//gremo naprej za x vrst
				napr_zacetek = true;
				//arewethereyet = false;
			}



		}




	}
        ros::spinOnce();
//	cout<<"grem malo spat"<<endl<<endl;
//        r.sleep();
    }


    return 0;
}
