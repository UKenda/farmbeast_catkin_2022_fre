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

float avg_right_scan, avg_left_scan;
int scan_count, best_angle_index;


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

float bestAngleIndexToRot(int angle)
    {
        int max_angle = 30; // najvecji kot, ki ga dosezejo kolesa je 40 stopinj - pozor: izmisljen podatek
        float rotation=0;
        float normalised_rotation= 0;

      /*  if(135-angle>max_angle)//desno
        {
            roatation = max_angle;
        }
        else if(135-angle<-max_angle)//levo
        {
            roatation = -max_angle;
        }
        else
        {
            rotation = 135-angle;
        }
*/
        angle = 135-angle;
        normalised_rotation = angle*1.0/max_angle*1.0;
        if(normalised_rotation<-1)
            normalised_rotation=-1;
        else if(normalised_rotation>1)
            normalised_rotation=1;

        return normalised_rotation;

    }


//CALLBACK
void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){

    points.DELETEALL;
    // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;





        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;
  /* if(robot_run){
        if(voznja_nacin == 0){*/
		    	//ROS_INFO("LaserCallBack Trigered!");
	float angle_inc = scan->angle_increment;
	float angle_min = scan->angle_min;
	float angle_max = scan->angle_max;
    //float[] scan_data = scan->ranges;
	//ROS_INFO("ANGLE:: Min %f - Max: %f || Angle inc: %f, scan_count: %i", angle_min, angle_max, angle_inc);//, (int)sizeof(scan_data));
/*
	for(int i=1; i<sizeof(scan_data); i++)
	{
		ROS_INFO("data: %f", scan_data[i]);
    }
*/
	int count_l = 0;
	int count_r = 0;

	float sum_left_scans = 0;
	float sum_right_scans = 0; 

	//left side data
	for(int l=45; l < 80; l++)
	{
		if((scan->ranges[l] > min_val)&&(scan->ranges[l] < max_val))
		{
			count_l = count_l + 1;
			sum_left_scans = sum_left_scans + scan->ranges[l];
		}		
	}
	
	for(int r=190; r < 225; r++)
	{
		if((scan->ranges[r] > min_val)&&(scan->ranges[r] < max_val))
		{
			count_r = count_r + 1;
			sum_right_scans = sum_right_scans + scan->ranges[r];
		}
	}

	avg_right_scan = sum_right_scans/count_r;
	avg_left_scan = sum_left_scans/count_l;
	if(count_r == 0)
	{
		avg_right_scan = -1;
	}
	if(count_l == 0)
	{
		avg_left_scan = -1;
	}


    //joze
    float min_width = 0.45; //robot je sirok 42// najmanjša širina proge
    int max_angle = 115; // največji dovoljen kot alfa in beta
    float max_length = 1.2; // ne gledamo dlje kot max_length porežemo vrednosti ki so večje
    int count_0 =0;
    int min_0 = 10;
    int index_max = 0;
    int index_last_non_zero=0;
    float angle_alfa;
    float angle_beta;
    int best_index = -1;
    float best_c_length=0;


    scan_count++;
    //ROS_INFO("-----  %i  -------", scan_count);

    for(int i=1;i<270;i++)
    {

        if(scan->ranges[i-1]==0 || scan->ranges[i-1]>max_length)
        {
            count_0++;
        }
        else
        {

            if(scan->ranges[i]>0 && scan->ranges[i]<max_length  && index_last_non_zero>0)
        {
                float a = scan->ranges[i];//pozor kaj se zgodi če je na začetku
                float b = scan->ranges[index_last_non_zero];
                int angle_gama = i-index_last_non_zero;

                float c = sqrt(pow(a,2) + pow(b,2) - ((2*a*b) * cos(angle_gama * PI / 180.0)));//izracunamo dolzino stranice c

                if(c>=min_width)
                {
                   // ROS_INFO("Sirina: %f na indexu: %i",c, i-(angle_gama/2));
                    angle_alfa = acos((pow(b,2) + pow(c,2) - pow(a,2))/((2*b*c)))/(PI/180);// /(PI/180) zato da dobimo stopinje in ne radianov
                    angle_beta = acos((pow(a,2) + pow(c,2) - pow(b,2))/((2*a*c)))/(PI/180);
                    //ROS_INFO("a:%f b:%f c:%f  Alfa:%f Beta:%f Gama:%i", a,b,c,angle_alfa,angle_beta,angle_gama); //count 0 je v bistvu kot gama v stopinjah

                    if(angle_alfa<max_angle && angle_beta<max_angle ) // idealno je da je vsota kotov 180, zato izberemo trikotnik, ki ima vsoto čim bližje 180
                    {


                        //ROS_INFO("Mozni kandidat - index: %i sirina: %f", i-(angle_gama/2), c);

                        if(c>best_c_length)
                        {
                             best_index = i-(angle_gama/2);
                             best_c_length = c;
                             //ROS_INFO("---Izbran kandidat - index: %i sirina: %f", best_index, c);
                        }
                    }

                }

                count_0 = 1;
        }


        if(scan->ranges[i]>0 && scan->ranges[i]<max_length)
        {
            index_last_non_zero = i;
        }


        }
    }

        if(best_index ==-1)
        {
            //ROS_INFO("Nisem nasel prave smeri: pelji naravnost");
            best_angle_index = -1;
	    nova = 0.0;
	    cout << "smer: " << nova << endl;

        }else
        {
		
            best_angle_index = best_index;
	    nova = bestAngleIndexToRot(best_angle_index)*koeficient;
	    cout << "smer: " << nova << endl;
	    if(nova > 0.4)
		nova = 0.4;
	    if(nova < -0.4)
		nova = -0.4;
        }


    //joze

    //9ROS_INFO("AVG: left: %f num: %i || right: %f num: %i", avg_left_scan, count_l, avg_right_scan, count_r);

    


	
/*
	
    }
   }*/
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
