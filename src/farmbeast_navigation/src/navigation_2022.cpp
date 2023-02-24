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


using namespace std;
using namespace geometry_msgs;

#define PI 3.14159265


//CMD_Vel
geometry_msgs::Twist CMD_MSG;
int izvajanje = 0;
bool loop = true;
bool konec_vrste = false;
char smer [10] = {'L','D','L','D','L','D','L','L','L','L'};
int stevilo_zavijanja = 0;
float hitrost = 0.3;

//CALLBACK
void LaserCB(const sensor_msgs::LaserScan::ConstPtr& scan){

    if(izvajanje == 0)
    {
        float desna = 0, leva = 0, smer = 0, sredina = 0.08;
        int delitelj = 0;
        for(int l = 0; l <= (scan->ranges.size()/2-scan->ranges.size()*sredina/2); l++){

            if(scan->ranges[l] < 2 && scan->ranges[l] > -2){
                //test
                desna +=  cos((21+l)*3.14/180) * scan->ranges[l];
                //staro
                
                //desna += scan->ranges[l];
                delitelj++;
            }
        }
        if (delitelj ==0) desna = 0;
        //else if (delitelj <5) desna = 0.63;
        else desna = desna / delitelj;
    
        cout << "Desna: " << desna <<endl;
        delitelj = 0;

        for(int l = (scan->ranges.size()/2+scan->ranges.size()*sredina/2); l <= scan->ranges.size(); l++){

            if(scan->ranges[l] < 2 && scan->ranges[l] > -2){
                leva +=  cos((21+(-1*(l-scan->ranges.size())))*3.14/180) * scan->ranges[l];
                //leva += scan->ranges[l];
                delitelj++;
                //cout << "range " << scan->ranges[l] <<endl;
                //cout << "kot: " << 21+(-1*(l-scan->ranges.size())) <<endl;
                //cout << "cos" << cos((21+(-1*(l-scan->ranges.size()))))<<endl;
                //cout << "Leva stran vmesne: " << cos((21+(-1*(l-scan->ranges.size())))) * scan->ranges[l] <<endl;

                
            }
        }
        if (delitelj ==0) leva = 0;
        //else if (delitelj <5) leva = 0.63;
        else leva = leva / delitelj;
        cout << "Leva: " << leva << endl;
        delitelj = 0;
        //cout << "velikost: " << scan->ranges.size() << endl;
    float x = (desna - leva)*1.3;
        //obe strani se zaznava
        if(leva > 0.001 && desna > 0.001){
            //smer = (desna - leva)*1.8;
            if(x>0.35)x=0.35;
            else if(x<-0.35)x=-0.35;
            cout << "x: " << x << endl;

            if(x >= 0)//x je pozitiven , desna je vecja razdalja pojdi desno
            {
                //cout << " x je pozitiven desno" <<endl;
                //hitrost = 0.3;
                smer = -0.3*x/0.25; // leva + |||||   denso -
            }
            else if (x < 0)//x je negativen, leva stran je vecja podji levo
            {
                //hitrost = 0.3;
                smer = - 0.3*x/0.25;
            }
            
            }
        else if(leva > 0.001){
            //hitrost = 0.3;
            cout << "samo leva stran"<< endl;

           x=(0.4-leva);
            if(x >= 0)//  desna vecja
            {
                //hitrost = 0.3;
                smer = - 0.3*x/0.35; // leva + |||||   denso -
            }
            else if (x < 0)//pojdi levo
            {
                //hitrost = 0.3;
                smer = - 0.3*x/0.35;
            }

        }
        else if(desna > 0.001){
            x=(desna-0.4);
            if(x >= 0)//  pojdi desno
            {
                //hitrost = 0.3;
                smer = -0.3*x/0.35; // leva + |||||   denso -
            }
            else if (x < 0)//pojdi levo
            {
                //hitrost = 0.3;
                smer =  -0.3*x/0.35;
            }
        }
        else{
            cout << "Konec vrste "<< endl;
            konec_vrste = true;

        }
        if(hitrost > 0.5) hitrost = 0.5;
        cout << "Smer: " << smer << "  Hitrost:  " << hitrost << endl;
        hitrost = 0.3;
        CMD_MSG.angular.z = smer;
        CMD_MSG.linear.x = hitrost;
        //loop=true;
    }
    else{

    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser");
    ros::NodeHandle nh;
    ros::Rate r(10);
    cout << "zdravo!" << endl;
    nh.setParam("/farmbeast/navigation",true);
    //sub/pub
    ros::Subscriber laser = nh.subscribe("/camera/scan",10, LaserCB);
    ros::Publisher premik = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    nh.setParam("/farmbeast/izvajanje",0);
    
    nh.setParam("/farmbeast/skropilnica/S1S2",true);

    cout << "pred whille" << endl;
    nh.getParam("/farmbeast/navigation",loop);
    while(loop){
        nh.getParam("/farmbeast/izvajanje",izvajanje);
        nh.getParam("/farmbeast/wheel/speed",hitrost);
        //loop=false;
        if(izvajanje==0) premik.publish(CMD_MSG);
        ros::spinOnce();
        r.sleep();
        nh.getParam("/farmbeast/navigation",loop);
        if (konec_vrste)
        {
                nh.setParam("/farmbeast/skropilnica/S1S2",true);

            konec_vrste=false;
            cout << "stevilo zavijanja" << stevilo_zavijanja<< endl;
            if(smer[stevilo_zavijanja] == 'L')
            {
                cout << "Grem LEVO!!!" << endl;
             nh.setParam("/farmbeast/izvajanje",2);
            }
            else if(smer[stevilo_zavijanja] == 'D')
            {
            cout << "Grem DESNO!!!" << endl;

            nh.setParam("/farmbeast/izvajanje",1);
            }
            stevilo_zavijanja++;
        }
    }
    cout << "konec" << endl;
    return 0;
}
