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

    bool calibrate_kot = false;
    bool poravnava = false;
    bool poravnava_zac = false;
    bool poravnava_kon = false;


    //IMU
    int smer = 0;
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

    //cas
   int naprejCas = 1000000;
   int stranCas = 1000000;
   bool gremoNaprej = true;
   bool gremoStran = false;
   bool obrni = false;
    
   int stevec_kolkic = 0;
    //CMD_Vel
    geometry_msgs::Twist CMD_MSG;
    float zSmer = 0.0;
    float xHitrost;
    float zHitrost;

    //marker
    visualization_msgs::Marker points;
    geometry_msgs::Point p;
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

void CompasCB(const sensor_msgs::Imu::ConstPtr& compas){
    //cout << "Orientacija x-osi: " << compas->orientation.x << endl;

    osx = compas->orientation.x;
    osy = compas->orientation.y;
    osz = compas->orientation.z;
    w = compas->orientation.w;

    tf::Quaternion q((double)osx, (double)osy, (double)osz, (double)w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    stopinje = (yaw*180.00)/3.14;

    if(calibrate_kot){
        centriran_kot = stopinje;
        calibrate_kot = false;
    }


    if(poravnava){
       if(smer == 1){
        if(!(stopinje <= centriran_kot + 1.0 && stopinje >= centriran_kot - 1.0)){
            if(centriran_kot > stopinje + 1.0){
                zSmer = -1*zHitrost;
                cout << centriran_kot << " " << stopinje << endl;
            }else{
                zSmer = zHitrost;
                cout << centriran_kot << " " << stopinje << endl;
            }
        }else{
            poravnava_kon = true;
        }
       }else{
           if(centriran_kot >= 0.0){
               centriran_kot = centriran_kot - 180.0;
           }else{
               centriran_kot = centriran_kot + 180.0;
           }

           if(!(stopinje <= centriran_kot + 1.0 && stopinje >= centriran_kot - 1.0)){
               if(centriran_kot > stopinje + 1.0){
                   zSmer = -1*zHitrost;
                   cout << centriran_kot << " " << stopinje << endl;
               }else{
                   zSmer = zHitrost;
                   cout << centriran_kot << " " << stopinje << endl;
               }
           }else{
               poravnava_kon = true;
           }
       }
    }else if(obrni){
        

        
        if(zacetek){


            if(stevec_kolkic == 0){
                if(smer == 1){ //0 = desno, 1 = levo
                    
                    if(stopinje > -100.0){
                        konc_stopinje = stopinje - 80.0;
                    }else{
                        konc_stopinje = 180.0 - (80.0 -(stopinje + 180.0));
                    }
                }else{ //desno

                    if(stopinje < 100.0){
                        konc_stopinje = stopinje + 80.0;
                    }else{
                        konc_stopinje = -180.0 + (80.0 + (stopinje - 180.0));
                    }

                }
                stevec_kolkic++;
            }else{
                if(smer == 1){ //1 = desno, 0 = levo

                    if(stopinje > -115.0){
                        konc_stopinje = stopinje - 65.0;
                    }else{
                        konc_stopinje = 180.0 - (65.0 -(stopinje + 180.0));
                    }
                }else{ //desno

                    if(stopinje < 115.0){
                        konc_stopinje = stopinje + 65.0;
                    }else{
                        konc_stopinje = -180.0 + (65.0 + (stopinje - 180.0));
                    }

                }
                stevec_kolkic = 0;
            }

            zacetek = false;
        }

        if(smer == 0){
            if(!(stopinje <= konc_stopinje + 1.0 && stopinje >= konc_stopinje - 1.0)){
                zSmer = -1*zHitrost;
                cout << konc_stopinje << " " << stopinje << endl;
            }else{
                if(step1 == false){
                    step1 = true;
                    gremoStran = true;
                    obrni = false;
                    cout << "STEP1" << endl;
                }else if(step2 == false){
                    step2 = true;
                    //gremoNaprej = true;
                    obrni = false;
                    konec = true;
                    cout << "STEP2" << endl;
                }else if(step2 == true && step1 == true){
                    konec = true;
                    obrni = false;
                    cout << "konec" << endl;
                }
            }
        }else{
            if(!(stopinje <= konc_stopinje + 1.0 && stopinje >= konc_stopinje - 1.0)){
                zSmer = 1 * zHitrost;
                cout << konc_stopinje << " " << stopinje << endl;
            }else{
                if(step1 == false){
		    
                    step1 = true;
                    gremoStran = true;
                    obrni = false;
                    cout << "STEP1" << endl;
                }else if(step2 == false){
                    step2 = true;
                    konec = true;
                    obrni = false;
                    cout << "STEP2" << endl;
                }else if(step2 == true && step1 == true){
                    konec = true;
                    obrni = false;
                    cout << "konec" << endl;
                }
            }
        }
    }

    //cout << "Roll: " << roll << " Pitch: " << pitch << " Yaw: " << (yaw*180.0)/3.14 << endl;





    /*oTren = compas->orientation.x;

    if(zacetek){
        oCenter = 0.0;
        zacetek = false;
        cout << "Zacetek smo nastavili na false" << endl;
    }else{
        oRazl = fabs(oTren - oPred);
        oCenter = oCenter + oRazl;
        if(smer == 0){ //ce moramo v desno
            if(oCenter <= o90){
                cout << "DESNO" << o90 << " " << oCenter << endl;
                zSmer = 0.2;
            }else{
                cout << "KONEC" << endl;
                konec = true;
            }
        }else{
            if(oCenter <= o90){
                cout << "LEVO" << -1*o90 << " " << oCenter << endl;
                zSmer = -0.2;
            }else{
                cout << "KONEC" << endl;
                konec = true;
            }
        }
*/


        /*gettimeofday(&kon, NULL);

        pre_pot = pre_pot + ((compas->linear_acceleration.x + pospesek)*(float)pow((float)(kon.tv_usec - zac.tv_usec)/1000000 + (float)(kon.tv_sec - zac.tv_sec),2))/2;
        cout << pre_pot << endl;*/
        //cout << compas->linear_acceleration.x << endl;
        //cout << oTrenutna << endl;

    //}
    //oPred = oTren;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "laser");

    ros::NodeHandle nh;

    ros::Subscriber compas = nh.subscribe("/imu/data", 10, CompasCB);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    nh.setParam("drive_mode_skeed",false);
    nh.setParam("drive_mode_ackerman",true);
    int nazaj_razd = 0;
    if(nh.hasParam("/PhidgetsImuNodelet/smer")){
        /*
    <param name="casNaprej" value="1000000"/>
    <param name="casStran" value="1000000"/>
    <param name="hitrost" value="0.3"/>
    <param name="hitrost_z" value = "0.2"/>

         */
        nh.getParam("/PhidgetsImuNodelet/o90", o90);
        nh.getParam("/PhidgetsImuNodelet/smer", smer);
        nh.getParam("/PhidgetsImuNodelet/casNaprej", naprejCas);
        nh.getParam("/PhidgetsImuNodelet/casStran", stranCas);
        nh.getParam("/PhidgetsImuNodelet/hitrost", xHitrost);
        nh.getParam("/PhidgetsImuNodelet/hitrost_z", zHitrost);
        nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);
        nh.getParam("/farmbeast_base_driver/wheel1_pos", wheel1_tren);
        nh.getParam("/farmbeast_base_driver/wheel2_pos", wheel2_tren);
        nh.getParam("/farmbeast_base_driver/wheel3_pos", wheel3_tren);
        nh.getParam("/farmbeast_base_driver/wheel4_pos", wheel4_tren);
        nh.getParam("/PhidgetsImuNodelet/napr_razd", napr_razd);
        nh.getParam("/PhidgetsImuNodelet/stran_razd", stran_razd);
       	nh.getParam("/PhidgetsImuNodelet/nazaj_razd", nazaj_razd); 

    }else{
        cout << "Ni naslo parametra" << endl;
    }
        cout << "Prisli smo cez branje parametrov" << endl;

     //nh.setParam("ackerman_drive_mode",3);

    double rate = 10.0;
    ros::Rate r(rate);


    points.header.frame_id = "/my_frame";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w  = 1.0;
	poravnava = false;
	gremoNaprej = true;
    //gremoNaprej = true;
    cout << "Smo pred zanko" << endl;
    while(ros::ok){

        marker_pub.publish(points);
        if(voznja_nacin == 1){
            nh.getParam("/farmbeast_base_driver/wheel1_pos", wheel1_tren);
            nh.getParam("/farmbeast_base_driver/wheel2_pos", wheel2_tren);
            nh.getParam("/farmbeast_base_driver/wheel3_pos", wheel3_tren);
            nh.getParam("/farmbeast_base_driver/wheel4_pos", wheel4_tren);
        if(gremoNaprej){
                if(napr_zacetek){
                    nh.setParam("ackerman_drive_mode",1); //dolocimo od kje bomo izhajali
                    //nh.setParam("/farmbeast_base_driver/odo_reset",1);
                    cout << "NAPREJ" << endl;
                    wheel1_pos = wheel1_tren;
                    wheel2_pos = wheel2_tren;
                    wheel3_pos = wheel3_tren;
                    wheel4_pos = wheel4_tren;
                    povprecje_pos = (wheel1_pos+wheel2_pos+wheel3_pos+wheel4_pos)>>2;
                    //povprecje_tren = povprecje_pos;
                    napr_zacetek = false;
                }
                
                povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
                cout << povprecje_pos << " " << povprecje_tren << endl;
                if(!(povprecje_tren >= povprecje_pos + napr_razd)){
                    CMD_MSG.linear.x = xHitrost;
                    CMD_MSG.angular.z = 0.0;
                    premik.publish(CMD_MSG);
                }else{
                    napr_konec = true;
                }




                if(napr_konec){
                    gremoNaprej = false;
                    obrni = true;
                    zacetek = true;
                    daj_na_ack = true;
                    napr_konec = false;
                    napr_zacetek = true;
                }
            }else if(gremoStran){
                if(stran_zacetek){
                    nh.setParam("ackerman_drive_mode",1);


                    cout << "STRAN" << endl;
                    wheel1_pos = wheel1_tren;
                    wheel2_pos = wheel2_tren;
                    wheel3_pos = wheel3_tren;
                    wheel4_pos = wheel4_tren;
                    povprecje_pos = (wheel1_pos+wheel2_pos+wheel3_pos+wheel4_pos)>>2;
                    //povprecje_tren = povprecje_pos;
                    stran_zacetek = false;
                }

                povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;
                cout << povprecje_pos << " " << povprecje_tren << endl;
                if(!(povprecje_tren>=povprecje_pos+ stran_razd)){
                    CMD_MSG.linear.x = xHitrost;
                    CMD_MSG.angular.z = 0.0;
                    premik.publish(CMD_MSG);
                }else{
                    stran_konec = true;
                }

                if(stran_konec){
                    gremoStran = false;
                    obrni = true;
                    zacetek = true;
                    daj_na_ack = true;
                    stran_konec = false;
                    stran_zacetek= true;
                }
            }else if(obrni && stevec % 5 == 0){
                if(daj_na_ack){
                    nh.setParam("ackerman_drive_mode",3);

                }
                CMD_MSG.angular.z = 0.0;
                zSmer = -zSmer;
                CMD_MSG.linear.x = zSmer;
                premik.publish(CMD_MSG);
                stara = zSmer;
            }else if(konec == true){
                if(konec_zac){
                    nh.setParam("ackerman_drive_mode",1); //dolocimo od kje bomo izhajali

                    cout << "KONEC" << endl;
                    wheel1_pos = wheel1_tren;
                    wheel2_pos = wheel2_tren;
                    wheel3_pos = wheel3_tren;
                    wheel4_pos = wheel4_tren;
                    povprecje_pos = (wheel1_pos+wheel2_pos+wheel3_pos+wheel4_pos)>>2;
                    //povprecje_tren = povprecje_pos;
                    konec_zac = false;
                }

                povprecje_tren = (wheel1_tren+wheel2_tren+wheel3_tren+wheel4_tren)>>2;

                if(!(povprecje_tren>=povprecje_pos + nazaj_razd)){
                    CMD_MSG.linear.x = xHitrost;
                    CMD_MSG.angular.z = 0.0;
                    premik.publish(CMD_MSG);
                }else{
                    konec_kon = true;
                }




                if(konec_kon){
                    konec_kon = false;
                    konec_zac = true;
                    nh.setParam("/PhidgetsImuNodelet/nac_voznje",0);
                    if(smer == 0)
                        nh.setParam("/PhidgetsImuNodelet/smer", 1);
                    else
                        nh.setParam("/PhidgetsImuNodelet/smer",0);
                   // poravnava = true;
                    gremoNaprej = true;
                    napr_zacetek = true;
                    konec = false;
                    step1 = false;
                    step2 = false;
                    obrni = false;
                    daj_na_ack = true;
                    cout << "KONCALI SMOOOO" << endl;
                    nh.getParam("/PhidgetsImuNodelet/smer",smer);


                }


                stevec1++;
            }

            stevec++;
        }

        nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);


        ros::spinOnce();
        r.sleep();
    }


    return 0;
}
