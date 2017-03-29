//
// Created by kelfor on 3/27/17.
//
#include <ros/ros.h>
#include<tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include<geometry_msgs/Twist.h>
//#include <main.h>
#include <nav_msgs/Odometry.h>
#include"asp.h"
#include "serial.h"
#define MAF_BUF_LEN 20

#define BUF_LEN 256


ros::Time current_time, last_time;
int cmd_vel_source = 0;// 0: From image processing 1: From Navigation node
typedef struct
{
    uint8_t dir;
    uint32_t cnt;
    uint32_t scl;
}Tri_t;


static uint32_t cnt = 0;
static Rmp_t rmp;
static Tri_t tri;//#include <main.h>
static Maf_t maf;
static float maf_buf[MAF_BUF_LEN];
static uint8_t dir = 0;

FIFO_t tx_fifo;
uint8_t tx_buf[2][BUF_LEN];
KylinMsg_t txKylinMsg;
uint8_t exit_flag = 0;

FIFO_t rx_fifo;
uint8_t rx_buf[2][BUF_LEN];

VirtualDBUS_t vdbus;
VirtualCBUS_t vcbus;
CalibMsg_t calibMsg;
MotorMsg_t motorMsg;
ZGyroMsg_t zgyroMsg;
OdomeMsg_t odomeMsg;
GraspMsg_t graspMsg;
KylinMsg_t kylinMsg;


void init()
{
    FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);
    FIFO_Init(&tx_fifo, tx_buf[0], BUF_LEN);
}
static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg)
{
    //printf("*************************************KYLIN********************************************\n");
    printf("fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->fs,
           kylinMsg->cp.x, kylinMsg->cp.y, kylinMsg->cp.z, kylinMsg->gp.e, kylinMsg->gp.c,
           kylinMsg->cv.x, kylinMsg->cv.y, kylinMsg->cv.z, kylinMsg->gv.e, kylinMsg->gv.c);
}

void Tri_Init(Tri_t* tri, uint32_t scl)
{
    tri->scl = scl;
    tri->cnt = 0;
    tri->dir = 0;
}
void PullMsg(ros::Publisher & odom_pub)
{
    // Get fifo free space
    //TODO: publish Odometry message.
    int len = FIFO_GetFree(&rx_fifo);
    // If fifo free space insufficient, pop one element out
    if (!len) {
        uint8_t b;
        len = FIFO_Pop(&rx_fifo, &b, 1);
    }
    // Read input stream according to the fifo free space left
    len = read_serial(rx_buf[1], len, 222);
    //printf("PULL LEN= %d\n", len);
    // Push stream into fifo
    FIFO_Push(&rx_fifo, rx_buf[1], len);
    // Check if any message received
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vrc, &vdbus.rcp)) {
        //VRC_Proc(&vdbus.rcp);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vhc, &vdbus.hcp)) {
        //VHC_Proc(&vdbus.hcp);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vdbus, &vdbus)) {
        //VDBUS_Proc(&vdbus);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vcbus, &vcbus)) {
        //VCBUS_Proc(&vcbus);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_calib, &calibMsg)) {
        //Dnl_ProcCalibMsg(&calibMsg);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_motor, &motorMsg)) {
        //Dnl_ProcMotorMsg(&motorMsg);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_zgyro, &zgyroMsg)) {
        //Dnl_ProcZGyroMsg(&zgyroMsg);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_odome, &odomeMsg)) {
        //Dnl_ProcOdomeMsg(&odomeMsg);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_grasp, &graspMsg)) {
        //Dnl_ProcGraspMsg(&graspMsg);
    }
    else if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &kylinMsg)) {
        //printf("Before pop -> size: %d, used: %d, free: %d\n", FIFO_GetSize(&rx_fifo), FIFO_GetUsed(&rx_fifo), FIFO_GetFree(&rx_fifo));
        //printf("Before pop: %d\n", FIFO_GetFree(&rx_fifo));
        Dnl_ProcKylinMsg(&kylinMsg);
        //printf("After pop: %d\n", FIFO_GetFree(&rx_fifo));
    }
    else {
        //uint8_t b;
        //len = FIFO_Pop(&rx_fifo, &b, 1);
    }


    tf::TransformBroadcaster odom_broadcaster;

    current_time = ros::Time::now();



    double theta = kylinMsg.cp.z/1000.0;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    /* 将里程计的偏航角转换成四元数，四元数效率高，这样使用二维和三维的功能包是一样的。*/
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    /*
Here we'll create a TransformStamped message that we will send out over tf. We want to publish the transform from the "odom" frame to the "base_link" frame at current_time. Therefore, we'll set the header of the message and the child_frame_id accordingly, making sure to use "odom" as the parent coordinate frame and "base_link" as the child coordinate frame.
*/

    odom_trans.transform.translation.x = kylinMsg.cp.x/1000.0; //Make sure the direction is correct
    odom_trans.transform.translation.y = kylinMsg.cp.y/1000.0;
    odom_trans.transform.transodom_publation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
/*
 在此处我们发布了当前最新base_link坐标系与odem坐标系之间的变换。
*/

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = kylinMsg.cp.x/1000.0;
    odom.pose.pose.position.y = kylinMsg.cp.y/1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /*
Here we fill in the transform message from our odometry data, and then send the transform using our TransformBroadcaster.
*/
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = kylinMsg.cv.x/1000.0;
    odom.twist.twist.linear.y = kylinMsg.cv.y/1000.0;
    odom.twist.twist.angular.z = kylinMsg.cv.z/1000.0;

    //publish the message
    odom_pub.publish(odom);

/*
我们发布nav_msgs/Odometry消息(odom)，从而导航包能从中获取机器人的速度。
我们设置child_frame_id为"base_link"坐标系，因为我们计算的速度才是base_link 坐标系下的速度。
*/

}

void PushMsg()
{
	//txKylinMsg.fs = 1;
	//txKylinMsg.cv.x = 2000;
	uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], & msg_head_kylin, &txKylinMsg);
	FIFO_Pop(&tx_fifo, tx_buf[1], len);
	write_serial(tx_buf[1], len, 50);
}
void cmd_velSourceCallback(const std_msgs::Int32 & source )
{
    cmd_vel_source = source.data;
}

void controlCmd_velFromeNavigationCallback(const geometry_msgs::Twist& cmd_vel)
{
    if(1 == cmd_vel_source)// Only valid when selected.
    {
        txKylinMsg.cp.x = 1000;
        txKylinMsg.cv.x = cmd_vel.linear.x;
        txKylinMsg.cp.y = 1000;
        txKylinMsg.cv.y = cmd_vel.linear.z;
        txKylinMsg.cp.z = 1000;
        txKylinMsg.cv.z = cmd_vel.angular.y;
    }


}

/* ---------------------------Kylinbot Control Callback Funcion -------------------------------*/
void controlCmd_velFromImageProcessingCallback(const geometry_msgs::Twist& cmd_vel)
{
//    ROS_INFO("Received a /cmd_vel message!");
  //  ROS_INFO("Linear Components:[%f,%f,%f]",cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
    //ROS_INFO("Angular Components:[%f,%f,%f]",cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);
    // Do velocity processing here:
    // Use the kinematics of your robot to map linear and angular velocities into motor commands
//    v_l = ...
//    v_r = ...
    if(0 == cmd_vel_source)  // Only valid when selected.
    {
        txKylinMsg.cp.x = cmd_vel.linear.x;
        txKylinMsg.cv.x = 1000;
        txKylinMsg.cp.y = cmd_vel.linear.z;
        txKylinMsg.cv.y = 1000;
        txKylinMsg.cp.z = cmd_vel.angular.y;
        txKylinMsg.cv.z = 1000;
        txKylinMsg.gp.e = cmd_vel.linear.y;
        txKylinMsg.gv.e = 1000; //cmd_vel.angular.y;
    }


    // Then set your wheel speeds (using wheel_left and wheel_right as examples)
//    wheel_left.set_speed(v_l)
//    wheel_right.set_speed(v_r)
}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "kylinbot_move");
    ros::NodeHandle n;

    ros::Subscriber sub_1 = n.subscribe("kylinbot/cmd_vel", 1000, controlCmd_velFromImageProcessingCallback);
    //ros::Publisher pub = n.advertise<nav_msgs::Odometry_>("kylinbot/Odom",100); //remain buggy.
    ros::Subscriber sub_2 = n.subscribe("cmd_vel",100,controlCmd_velFromeNavigationCallback);
    ros::Subscriber sub_3 = n.subscribe("kylinbot/cmd_vel_source",100,cmd_velSourceCallback);


    //Publish Odom

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("kylinbo/odom", 100);
    ros::Rate r(100);


    init();
    uint32_t cnt = 0;
    Rmp_Config(&rmp, 50000);
    Maf_Init(&maf, maf_buf, MAF_BUF_LEN);
    Tri_Init(&tri, 2.5e4);

    //char* device = argv[1];
    const char* device = "/dev/ttyTHS2";
    if (connect_serial(device,115200) == -1)
    {
        printf("serial open error!\n");
        return -1;
    }
    while(ros::ok())
    {
        ros::spinOnce();
        PullMsg(odom_pub);
		PushMsg();
        r.sleep();
    }

    return 0;
}