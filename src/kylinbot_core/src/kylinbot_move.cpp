//
// Created by kelfor on 3/27/17.
//
#include <ros/ros.h>
#include<tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include<geometry_msgs/Twist.h>
//#include <main.h>
#include <nav_msgs/Odometry.h>
#include"asp.h"
#include "serial.h"
#include <iostream>
#define MAF_BUF_LEN 200
#include<kylinbot_core/kylin_vrc.h>
#include <main.h>

#define BUF_LEN 256
using namespace std;

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

ZGyroMsg_t zgyroMsg;
KylinMsg_t kylinMsg;

PosCalibMsg_t posCalibMsg;
Sr04sMsg_t sr04sMsg;
VirtualRC_t vrc;
VirtualRC_t vrc_mimic;
void initFIFO()
{
    //	printf("rx add:%d, rx size: %d\n",rx_fifo.m,rx_fifo.s);
    FIFO_Init(&rx_fifo, rx_buf[0], BUF_LEN);
    //	printf("rx add:%d, rx size: %d\n",rx_fifo.m,rx_fifo.s);
    //printf("tx add:%d, tx size: %d\n",tx_fifo.m,tx_fifo.s);
    FIFO_Init(&tx_fifo, tx_buf[0], BUF_LEN);
    //printf("tx add:%d, tx size: %d\n",tx_fifo.m,tx_fifo.s);
}

static void Dnl_ProcKylinMsg(const KylinMsg_t* kylinMsg,ros::Publisher & odom_pub)
{
    //printf("*************************************KYLIN********************************************\n");
    printf("fs=%x,px=%d,py=%d,pz=%d,pe=%d,pc=%d,vx=%d,vy=%d,vz=%d,ve=%d,vc=%d\n", kylinMsg->cbus.fs,
           kylinMsg->cbus.cp.x, kylinMsg->cbus.cp.y, kylinMsg->cbus.cp.z, kylinMsg->cbus.gp.e, kylinMsg->cbus.gp.c,
           kylinMsg->cbus.cv.x, kylinMsg->cbus.cv.y, kylinMsg->cbus.cv.z, kylinMsg->cbus.gv.e, kylinMsg->cbus.gv.c);
}

static void Dnl_ProcVirtualRC(const VirtualRC_t* vrc,ros::Publisher& vrc_pub)
{
    kylinbot_core::kylin_vrc vrc_msg;
    vrc_msg.frame_id = vrc->frame_id;
    memcpy(vrc_msg.buf.elems,vrc->buf,6*sizeof(uint8_t));


    vrc_pub.publish(vrc_msg);


}

void Tri_Init(Tri_t* tri, uint32_t scl)
{
    tri->scl = scl;
    tri->cnt = 0;
    tri->dir = 0;
}
void PullMsg(ros::Publisher & odom_pub,ros::Publisher & vrc_pub)
//void PullMsg()
{
    // Get fifo free space
    int len = FIFO_GetFree(&rx_fifo);
    // If fifo free space insufficient, pop one element out
    if (!len) {
        uint8_t b;
        len = FIFO_Pop(&rx_fifo, &b, 1);
    }
    // Read input stream according to the fifo free space left
    len = read_serial(rx_buf[1], len, 255);
    // Push stream into fifo
    FIFO_Push(&rx_fifo, rx_buf[1], len);
    // Check if any message received
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_kylin, &kylinMsg)) {
        Dnl_ProcKylinMsg(&kylinMsg,odom_pub);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_sr04s, &sr04sMsg)) {
        //Dnl_ProcSr04sMsg(&sr04sMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_zgyro, &zgyroMsg)) {
        //Dnl_ProcZGyroMsg(&zgyroMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_pos_calib, &posCalibMsg)) {
        //Dnl_ProcPosCalibMsg(&posCalibMsg);
    }
    if (Msg_Pop(&rx_fifo, rx_buf[1], &msg_head_vrc, &vrc)) {
        Dnl_ProcVirtualRC(&vrc,vrc_pub);
    }
    // printf("Push in PULL tx: add %d size: %d",tx_fifo.m, tx_fifo.s);
    // cout<<"pull 2.5"<<endl;
    //printf("Push in PULL tx: add %d size: %d",tx_fifo.m, tx_fifo.s);
    tf::TransformBroadcaster odom_broadcaster,kylinbot_move_broadcaster;

    current_time = ros::Time::now();



    double theta = kylinMsg.cbus.cp.z/1000.0;
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    /* 将里程计的偏航角转换成四元数，四元数效率高，这样使用二维和三维的功能包是一样的。*/
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans, map_to_odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";


    /*
Here we'll create a TransformStamped message that we will send out over tf. We want to publish the transform from the "odom" frame to the "base_link" frame at current_time. Therefore, we'll set the header of the message and the child_frame_id accordingly, making sure to use "odom" as the parent coordinate frame and "base_link" as the child coordinate frame.
*/


    odom_trans.transform.translation.x = kylinMsg.cbus.cp.x/1000.0; //Make sure the direction is correct
    odom_trans.transform.translation.y = kylinMsg.cbus.cp.y/1000.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);


//Assume map and odom need no tf.
    kylinbot_move_broadcaster.sendTransform(
            tf::StampedTransform (

                    tf::Transform ( tf::Quaternion ( 0, 0, 0 ), tf::Vector3 ( 0.0, 0.0, 0.0 ) ),
                    current_time,"map", "odom" ) );
/*
 在此处我们发布了当前最新base_link坐标系与odem坐标系之间的变换。
*/

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = kylinMsg.cbus.cp.x/1000.0;
    odom.pose.pose.position.y = kylinMsg.cbus.cp.y/1000.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    /*
Here we fill in the transform message from our odometry data, and then send the transform using our TransformBroadcaster.
*/
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = kylinMsg.cbus.cv.x/1000.0;
    odom.twist.twist.linear.y = kylinMsg.cbus.cv.y/1000.0;
    odom.twist.twist.angular.z = kylinMsg.cbus.cv.z/1000.0;

    //publish the message
    odom_pub.publish(odom);

/*
我们发布nav_msgs/Odometry消息(odom)，从而导航包能从中获取机器人的速度。
我们设置child_frame_id为"base_link"坐标系，因为我们计算的速度才是base_link 坐标系下的速度。
*/
    //printf("Push in PULL tx: add %d size: %d",tx_fifo.m, tx_fifo.s);
}

void PushMsg()
{
    uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], & msg_head_kylin, &txKylinMsg);
    FIFO_Pop(&tx_fifo, tx_buf[1], len);
    write_serial(tx_buf[1], len, 255);
}

void cmd_velSourceCallback(const std_msgs::Int8 & source )
{
    cmd_vel_source = source.data;
}


void mimicRCCallback(const kylinbot_core::kylin_vrc & vrc_msg)
{

    if(3 == cmd_vel_source)
    {
        vrc_mimic.frame_id = vrc_msg.frame_id;
        memcpy(vrc_mimic.buf,vrc_msg.buf.elems,6*sizeof(uint8_t));
        uint32_t len = Msg_Push(&tx_fifo, tx_buf[1], & msg_head_kylin, &vrc_mimic);
        FIFO_Pop(&tx_fifo, tx_buf[1], len);
        write_serial(tx_buf[1], len, 255);

    }

}


void controlCmd_velFromeNavigationCallback(const geometry_msgs::Twist& cmd_vel)
{
    if(1 == cmd_vel_source)// Only valid when selected.
    {
        txKylinMsg.cbus.cp.x = 2000;
        txKylinMsg.cbus.cv.x = cmd_vel.linear.x;
        txKylinMsg.cbus.cp.y = 2000;
        txKylinMsg.cbus.cv.y = cmd_vel.linear.z;
        txKylinMsg.cbus.cp.z = 2000;
        txKylinMsg.cbus.cv.z = cmd_vel.angular.y;

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
        txKylinMsg.cbus.cp.x = cmd_vel.linear.x;
        txKylinMsg.cbus.cv.x = 1000;
        txKylinMsg.cbus.cp.y = cmd_vel.linear.z;
        txKylinMsg.cbus.cv.y = 1000;
        txKylinMsg.cbus.cp.z = cmd_vel.angular.y;
        txKylinMsg.cbus.cv.z = 1000;
        txKylinMsg.cbus.gp.e = cmd_vel.linear.y;
        txKylinMsg.cbus.gv.e = 1000; //cmd_vel.angular.y;

    }


//  

    // Then set your wheel speeds (using wheel_left and wheel_right as examples)
//    wheel_left.set_speed(v_l)
//    wheel_right.set_speed(v_r)
}


class kylinbot_action
{
public:
    kylinbot_action(ros::NodeHandle &);
    bool isRunning();
    bool isPaused();
    bool isFinished();
private:
    bool isBusy;
    ros::Subscriber cmdSub;
    ros::Publisher  actionState_pub;

};


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "kylinbot_move");
    ros::NodeHandle n;

    ros::Subscriber sub_1 = n.subscribe("kylinbot/cmd_vel", 1000, controlCmd_velFromImageProcessingCallback);
    //ros::Publisher pub = n.advertise<nav_msgs::Odometry_>("kylinbot/Odom",100); //remain buggy.
    ros::Subscriber sub_2 = n.subscribe("cmd_vel",100,controlCmd_velFromeNavigationCallback);
    ros::Subscriber sub_3 = n.subscribe("kylinbot/cmd_vel_source",100,cmd_velSourceCallback);



    ros::Subscriber sub_4 = n.subscribe("kylinbot/mimicRC",100,mimicRCCallback);
    //Publish Odom

    //Virtual Control Command Publisher
    ros::Publisher vrc_pub = n.advertise<kylinbot_core::kylin_vrc>("vrc",100);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("kylinbo/odom", 100);
    ros::Rate r(100);


    //uint8_t * savePtr = NULL;
    initFIFO();
    //savePtr = tx_fifo.m;
    //uint32_t cnt = 0;
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
   // printf("add: %d", tx_fifo.m);

    while(ros::ok())
    {
        ros::spinOnce();
        PullMsg(odom_pub,vrc_pub);
        PushMsg();
        r.sleep();
    }

    return 0;
}
