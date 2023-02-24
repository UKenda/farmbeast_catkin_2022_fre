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

    osx = compas->orientation.x;
    osy = compas->orientation.y;
    osz = compas->orientation.z;
    w = compas->orientation.w;

    tf::Quaternion q((double)osx, (double)osy, (double)osz, (double)w);
    tf::Matrix3x3 m(q);

    m.getRPY(roll, pitch, yaw);

    stopinje = (yaw*180.00)/3.14;

  /*  if(calibrate_kot){
        centriran_kot = stopinje;
        calibrate_kot = false;
    }*/

	if(voznja_nacin == 1 || voznja_nacin == 3){
	poprvi = true;

	if(calibrate_kot){
                if(smer == 1){ //1 = desno, 0 = levo... v if stavek napises 180 - zeljen kot... znotraj pa zeljen kot... tam ko ni 180 napisano seveda :P
//TO-DO: CE MORAMO IT DIREKT V NASLEDNJO VRSTO, DA SE NE OBRACAMO ZA 90 STOPINJ AMPAK TAK KOT PRI TASK1
                    if(stopinje > -105.0){
                        konc_stopinje = stopinje - 75.0;
                    }else{
                        konc_stopinje = 180.0 - (75.0 -(stopinje + 180.0));
                    }
                }else{
                    if(stopinje < 105.0){
                        konc_stopinje = stopinje + 75.0;
                    }else{
                        konc_stopinje = -180.0 + (75.0 + (stopinje - 180.0));
                    }

                }
	calibrate_kot = false;
	}

        if(smer == 0){
            if(!(stopinje <= konc_stopinje + 5.0 && stopinje >= konc_stopinje - 5.0)){

//                cout << konc_stopinje << " " << stopinje << endl;
            }else{
               //na koncu smo
		ROS_INFO("TU SMO LEVO");
		arewethereyet = true;
	       //cout << "TU SMO DESNO" << endl;
            }
        }else{
            if(!(stopinje <= konc_stopinje + 5.0 && stopinje >= konc_stopinje - 5.0)){

  //              cout << konc_stopinje << " " << stopinje << endl;
            }else{
		//na koncu smo
		ROS_INFO("TU SMO DESNO");
		arewethereyet = true;
	      //cout << "TU SMO LEVO" << endl;
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
}


int main(int argc, char **argv){
    ros::init(argc, argv, "laser");

    ros::NodeHandle nh;

int value;
    ifstream myFile;
    myFile.open("/home/pi/PKP2017/src/Farmbeast/farmbeast_navigation/launch/task2_nav.txt");
    if (myFile.is_open())
        {
            cout << "notri smo" << endl;
            while(myFile>>value){
                //myFile>>value;
                cout << "Prebrali smo nekaj " << value << endl;
                branje.push_back(value);

            }

            myFile.close();
        }else{
        cout << "Ni datoteke" << endl;

    }

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
        nh.getParam("/PhidgetsImuNodelet/o90", o90);
        nh.getParam("/PhidgetsImuNodelet/smer", smer);
        nh.getParam("/PhidgetsImuNodelet/napr_razd", napr_razd);
        nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);
        nh.getParam("/farmbeast_base_driver/wheel1_pos", wheel1_tren);
        nh.getParam("/farmbeast_base_driver/wheel2_pos", wheel2_tren);
        nh.getParam("/farmbeast_base_driver/wheel3_pos", wheel3_tren);
        nh.getParam("/farmbeast_base_driver/wheel4_pos", wheel4_tren);
	nh.getParam("/farmbeast_auto", robot_run);
        nh.getParam("/fb_task3_left/zaznali",zaznaliLevo);
        nh.getParam("/fb_task3_right/zaznali",zaznaliDesno);
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
        nh.getParam("/fb_task3_left/zaznali",zaznaliLevo);
        nh.getParam("/fb_task3_right/zaznali",zaznaliDesno);
	if(poprvi){
        if(zaznaliDesno == true){
         nh.setParam("/BeepSek",2);
         usleep(2800000);
         nh.setParam("/Skropilnica/ventili",-1);
         zaznaliDesno = false;
         nh.setParam("/fb_task3_right/zaznali",zaznaliDesno);
        }else if (zaznaliLevo == true){
           nh.setParam("/BeepSek",2);
           usleep(2800000);
           zaznaliLevo = false;
           nh.setParam("/fb_task3_left/zaznali",zaznaliLevo);
           nh.setParam("/Skropilnica/ventili",1);
        }
}
	if(robot_run){
	//cout <<"V robot_run"<<endl;
		star_nacin = voznja_nacin;
		nh.getParam("/PhidgetsImuNodelet/nac_voznje", voznja_nacin);
		if(star_nacin == 2 && voznja_nacin == 0){
			i--;
			calibrate_kot = true;
			arewethereyet = false;
		}else if(star_nacin == 3 && voznja_nacin == 0 && arewethereyet == false){
			calibrate_kot = true;
			i--;
		}else if(star_nacin == 1 && voznja_nacin == 0){
			i--;
			calibrate_kot = true;
			arewethereyet = false;
		}
		if(voznja_nacin == 1){
		if(napr_zacetek){
			    if(branje[i]/10 == 2){
		                nh.setParam("/PhidgetsImuNodelet/smer",0);
				smer = 0;
		            }else{
		                nh.setParam("/PhidgetsImuNodelet/smer",1);
				smer = 1;
		            }

		            st_vrst = branje[i]%10; //ce gremo v naseldnjo takoj, mora biti 0
		            i++;
		    		cout << smer << " "  << st_vrst;
				napr_zacetek = false;
		}


		//cout <<"v voznja_nacin" <<endl;
			nh.getParam("/PhidgetsImuNodelet/smer", smer);
			if(arewethereyet == false){
		//cout<<"we are not there yet"<<endl;
			//spam ang > 0 
				    //CMD_MSG.linear.x = 0.0; 1.0 gremo v levo, -1.0 gremo v desno
				    if(smer == 1){
					    CMD_MSG.angular.z = -1.0;
					    premik.publish(CMD_MSG);
					    //ROS_INFO("TU se obracamo desno");
				    }
				    else{
					    CMD_MSG.angular.z = 1.0;
					    premik.publish(CMD_MSG);
					    //ROS_INFO("TU se obracamo levo");
				    }
			}else{
			//cout<<"we are there, yet"<<endl;
			//spam ang = 0
				nh.setParam("/drive_mode_skeed",true);
				nh.setParam("/drive_mode_ackerman",false);
				for(int i = 0; i<10; i++){
					CMD_MSG.linear.x = 0.0;
					CMD_MSG.angular.z = 0.0;
					premik.publish(CMD_MSG);
				}
				ROS_INFO("TU SMO se ustavili");
				//nh.setParam("/PhidgetsImuNodelet/nac_voznje", 0);
				voznja_nacin = 2;
				arewethereyet = false;
				usleep(1000000);
				//
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 2);//gremo naprej za x vrst
				/*if(smer == 0){
					smer = 1;
				}else{
					smer = 0;
                                }*/
				//nh.setParam("/PhidgetsImuNodelet/smer", smer);

				calibrate_kot = true;
				napr_zacetek = true;
				//arewethereyet = false;
			}
		}else if(voznja_nacin == 2){
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
			if((povprecje_tren <= povprecje_pos + napr_razd*st_vrst - 100)){
			    CMD_MSG.linear.x = 0.3;
			    CMD_MSG.angular.z = 0.0;
			    premik.publish(CMD_MSG);
			}else{
				CMD_MSG.linear.x = 0.0;
				CMD_MSG.angular.z = 0.0;
				premik.publish(CMD_MSG);
				usleep(1000000);

				nh.setParam("/drive_mode_skeed",false);
				nh.setParam("/drive_mode_ackerman",true);
				
				ROS_INFO("TU SMO se ustavili");
				//nh.setParam("/PhidgetsImuNodelet/nac_voznje", 0);
				voznja_nacin = 3;
				arewethereyet = false;
				//usleep(1000000);
				//
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 3);//gremo naprej za x vrst
				/*if(smer == 0){
					smer = 1;
				}else{
					smer = 0;
                                }*/
				//nh.setParam("/PhidgetsImuNodelet/smer", smer);

				calibrate_kot = true;
				napr_zacetek = true;
				//arewethereyet = false;
			}



		}else if(voznja_nacin == 3){
			//cout <<"v voznja_nacin" <<endl;
			nh.getParam("/PhidgetsImuNodelet/smer", smer);
			if(arewethereyet == false){
		//cout<<"we are not there yet"<<endl;
			//spam ang > 0 
				    //CMD_MSG.linear.x = 0.0; 1.0 gremo v levo, -1.0 gremo v desno
				    if(smer == 1){
					    CMD_MSG.angular.z = -1.0;
					    premik.publish(CMD_MSG);
					    //ROS_INFO("TU se obracamo desno");
				    }
				    else{
					    CMD_MSG.angular.z = 1.0;
					    premik.publish(CMD_MSG);
					    //ROS_INFO("TU se obracamo levo");
				    }
			}else{
			//cout<<"we are there, yet"<<endl;
			//spam ang = 0
				nh.setParam("/drive_mode_skeed",true);
				nh.setParam("/drive_mode_ackerman",false);
				for(int i = 0; i<10; i++){
					CMD_MSG.linear.x = 0.0;
					CMD_MSG.angular.z = 0.0;
					premik.publish(CMD_MSG);
				}
				ROS_INFO("TU SMO se ustavili");
				//nh.setParam("/PhidgetsImuNodelet/nac_voznje", 0);
				voznja_nacin = 0;
				arewethereyet = false;
				usleep(1000000);
				//
				nh.setParam("/PhidgetsImuNodelet/nac_voznje", 0);//gremo dalje skozi vrsto
				/*if(smer == 0){
					smer = 1;
				}else{
					smer = 0;
                                }*/
				//nh.setParam("/PhidgetsImuNodelet/smer", smer);

				calibrate_kot = true;
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
