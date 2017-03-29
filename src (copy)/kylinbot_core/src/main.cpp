//
// Created by kelfor on 3/27/17.
//
#include <ros/ros.h>
#include<tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include<geometry_msgs/Twist.h>
//#include <main.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include"asp.h"
#include "serial.h"
#define MAF_BUF_LEN 20

bool go_near_cube()
{
    //For Demo Use
    //Go straight, 2m ahead;
    return true;//TODO:
}
bool grab_cube()
{
    return true;//TODO:
}
bool fold_cubes()
{
    return true;//TODO:
}
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "kylinbot_main");
    ros::NodeHandle n;



    ros::Rate r(100);


  //  current_time = ros::Time::now();
//    last_time = ros::Time::now();

    int cmd_vel_source = 1;
    int workState = 0;
    while(ros::ok())
    {



        switch (workState)
        {
            case 0: // 
                //;//Ignore the cmd_vel from image processing.
                if(go_near_cube()) //If this is a loop, make sure you call ros::spinOnce()!
                    workState = 1;
                break;
            case 1:
                if(grab_cube())
                    workState = 2;
                break;
            case 2:
                break;
            case 3:
                break;
            case 100:
                fold_cubes();
                break;
            default:
                break;
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}