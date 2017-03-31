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
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include"asp.h"
#include "serial.h"
#define MAF_BUF_LEN 20
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool go_near_cube(MoveBaseClient & ac,move_base_msgs::MoveBaseGoal & goal,float x, float y, float z = 0.0,float yaw =0.0,
float pitch = 0.0, float roll = 0.0)
{
    //For Demo Use
    //Go straight, 2m ahead;

    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.w = yaw;

    ROS_INFO("Sent Goal: X; %4f  Y:%4f Z:%4f yaw:%4f pitch:%4f roll:%4f",x,y,z,yaw,pitch,roll);

    ac.sendGoal ( goal );
    bool result = ac.waitForResult(ros::Duration(10.0));//wait 10s.

    return result;//TODO:
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
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;



   // ros::init ( argc, argv, "navigation_goals" );
   // ros::NodeHandle n;
   // ros::Subscriber obstacle_distance_sub = n.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

   // ros::Subscriber start_searching_sub = n.subscribe ( "/dji_sdk_demo/start_searching",10,start_searching_callback );
    //Test filtering

    //tell the action client that we want to spin a thread by default

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";


    int x = 0;
   // while ( start_searching==false)
    //{

    sleep ( 10);

       // ros::spinOnce();
   // }
    MoveBaseClient ac ( "move_base", true );

    //wait for the action server to come up
   if(!ac.waitForServer ( ros::Duration ( 5.0 ) ) )
    {
        ROS_INFO ( "Movebase server failed to come up." );

        return -1;//ros::spinOnce();
    }




    ros::Rate r(100);


  //  current_time = ros::Time::now();
//    last_time = ros::Time::now();

   // int cmd_vel_source = 1;
    int workState = 0;
    while(ros::ok())
    {



        switch (workState)
        {
            case 0: // 
                //;//Ignore the cmd_vel from image processing.
                if(go_near_cube(ac,goal,4,3))
                    workState = 1;
                break;
            case 1:
                //
                ROS_INFO("Last Goal accomplished.");

                break;

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