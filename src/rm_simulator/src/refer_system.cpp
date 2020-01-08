#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "obstacle_detector/Obstacles.h"
#include "rm_simulator/ReferSystemInfo.h"
#include "rm_simulator/GameState.h"
#include "roborts_msgs/TwistAccel.h"

#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <list>
#define PI acos(-1)
#define f1X 0.5
#define f1Y 3.360
#define f2X 1.900
#define f2Y 1.935
#define f3X 4.100
#define f3Y 4.590
#define f4X 7.600
#define f4Y 1.740
#define f5X 6.200
#define f5Y 3.165
#define f6X 4.000
#define f6Y 0.510

bool isGameStart = false;

class ReferSystem
{
public:
    ReferSystem(ros::NodeHandle& nodehandle) :
    nh_(nodehandle)    
    {
        game_server = nh_.advertiseService("start_game", &ReferSystem::startGame,this);
        state_pub = nh_.advertise<rm_simulator::ReferSystemInfo>("refer_system_info", 1, true);

        for (int i = 0; i < 4; ++i) {
            cmd_pub[i]   = nh_.advertise<geometry_msgs::Twist>("/jackal"+std::to_string(i)+"/cmd_vel", 100);
            shoot_sub[i] = nh_.subscribe("/jackal"+std::to_string(i)+"/shoot", 10, &ReferSystem::updateShoot, this);
            refer_system_msg.robotHealth[i]=2000;
            refer_system_msg.robotShoot[i]=1;
            refer_system_msg.robotMove[i]=1;
        }
        obstacle[0] = nh_.subscribe("/jackal"+std::to_string(0)+"/obstacle_filtered", 10, &ReferSystem::obstacleCallBack0, this);
        obstacle[1] = nh_.subscribe("/jackal"+std::to_string(1)+"/obstacle_filtered", 10, &ReferSystem::obstacleCallBack1, this);
        obstacle[2] = nh_.subscribe("/jackal"+std::to_string(2)+"/obstacle_filtered", 10, &ReferSystem::obstacleCallBack2, this);
        obstacle[3] = nh_.subscribe("/jackal"+std::to_string(3)+"/obstacle_filtered", 10, &ReferSystem::obstacleCallBack3, this);
        acc_sub[0]  = nh_.subscribe("/jackal"+std::to_string(0)+"/cmd_vel_acc", 10, &ReferSystem::cmdCallback0, this);
        acc_sub[1]  = nh_.subscribe("/jackal"+std::to_string(1)+"/cmd_vel_acc", 10, &ReferSystem::cmdCallback1, this);
        acc_sub[2]  = nh_.subscribe("/jackal"+std::to_string(2)+"/cmd_vel_acc", 10, &ReferSystem::cmdCallback2, this);
        acc_sub[3]  = nh_.subscribe("/jackal"+std::to_string(3)+"/cmd_vel_acc", 10, &ReferSystem::cmdCallback3, this);

        refer_system_msg.header.frame_id = "/map";
        refer_system_msg.remain_bullet[0]=50;
        refer_system_msg.remain_bullet[2]=50;
        
        refer_system_msg.cellX[0]=f1X;
        refer_system_msg.cellY[0]=f1Y;
        refer_system_msg.cellX[3]=f4X;
        refer_system_msg.cellY[3]=f4Y;
        refer_system_msg.cellX[1]=f2X;
        refer_system_msg.cellY[1]=f2Y;
        refer_system_msg.cellX[4]=f5X;
        refer_system_msg.cellY[4]=f5Y;
        refer_system_msg.cellX[2]=f3X;
        refer_system_msg.cellY[2]=f3Y;
        refer_system_msg.cellX[5]=f6X;
        refer_system_msg.cellY[5]=f6Y;

        for (int i = 0; i < 6; ++i) {   
            for (int j = 0; j < 7; ++j) {
                refer_system_msg.cellState[i].state[j]=0;
            }
        }
    }

    void initialize(){
        for (int i = 0; i < 4; ++i) {
            refer_system_msg.robotHealth[i]=2000;
            refer_system_msg.robotShoot[i]=1;
            refer_system_msg.robotMove[i]=1;
            refer_system_msg.robotMoveDebuffTime[i]=0.;
            refer_system_msg.robotShootDebuffTime[i]=0.;
        }
        refer_system_msg.remain_bullet[0]=50;
        refer_system_msg.remain_bullet[2]=50;
        start_secs=ros::Time::now().toSec();
    }

    bool startGame(rm_simulator::GameState::Request &req,
                   rm_simulator::GameState::Response &res){
        isGameStart = req.start;
        res.received = true;
        if(isGameStart == true){
            initialize();
        }
        return true;
    }

    void updateShoot(const std_msgs::String::ConstPtr msg){
        // ROS_INFO_STREAM("center.x "<<filter_msg.circles.x);
        // data formate: a to b ( e.g. "0to3")
        if(msg->data.length() == 4){
            int targetIndex = msg->data[3]-'0';
            int selfIndex = msg->data[0]-'0';
            // bullet should > 0 and do not have shoot debuff!
            if(refer_system_msg.remain_bullet[msg->data[0]-'0']>0 && refer_system_msg.robotShoot[selfIndex]==1){
                refer_system_msg.remain_bullet[msg->data[0]-'0'] --; 

                int meetEnemy = 0;
                for(auto it_c = filter_msg[selfIndex].circles.begin() ; it_c!=filter_msg[selfIndex].circles.end(); it_c++){
                    if ( pow(it_c->center.x-robot_status[targetIndex].getOrigin().x(),2)+pow(it_c->center.y-robot_status[targetIndex].getOrigin().y(),2) < pow(it_c->radius,2)+0.1){
                        meetEnemy = 1;
                    }
                    // ROS_INFO_STREAM("distance^2 "<<pow(it_c->center.x-robot_status[targetIndex].getOrigin().x(),2)+pow(it_c->center.y-robot_status[targetIndex].getOrigin().y(),2));
                    // ROS_INFO_STREAM("radius^2+1 "<<pow(it_c->radius,2));
                    // ROS_INFO_STREAM("center.x "<<it_c->center.x);
                    // ROS_INFO_STREAM("center.x "<<it_c->radius);
                }
                if( meetEnemy==1 ){
                    float theta =  atan2(robot_status[targetIndex].getOrigin().y()-robot_status[selfIndex].getOrigin().y(),robot_status[targetIndex].getOrigin().x()-robot_status[selfIndex].getOrigin().x());
                    float thetaRelated = tf::getYaw(robot_status[targetIndex].getRotation()) - theta;
                    if(thetaRelated > PI) thetaRelated -= 2*PI;
                    if(thetaRelated < -PI) thetaRelated += 2*PI;
                    
                    thetaRelated = fabs(thetaRelated);
                    if(thetaRelated <= 45*PI/180) {
                        if(refer_system_msg.robotHealth[msg->data[3]-'0'] - 60 >=2000){ //uint format property
                            refer_system_msg.robotHealth[msg->data[3]-'0']=0;
                            //disable move and shoot
                            refer_system_msg.robotMove[msg->data[3]-'0'] = 0;
                            refer_system_msg.robotShoot[msg->data[3]-'0'] = 0;
                            ROS_INFO_STREAM("Died!");
                        }
                        else refer_system_msg.robotHealth[msg->data[3]-'0'] -= 60; // hit the back
                        
                        ROS_INFO_STREAM("Hit the back!!!");
                    }
                    else if (thetaRelated <= 135*PI/180) {

                        if(refer_system_msg.robotHealth[msg->data[3]-'0'] - 40 >=2000){
                            refer_system_msg.robotHealth[msg->data[3]-'0']=0;
                            //disable move and shoot
                            refer_system_msg.robotMove[msg->data[3]-'0'] = 0;
                            refer_system_msg.robotShoot[msg->data[3]-'0'] = 0;
                            ROS_INFO_STREAM("Died!");
                        }
                        else refer_system_msg.robotHealth[msg->data[3]-'0'] -= 40; // hit the side
                        ROS_INFO_STREAM("Hit the side!!");
                    }
                    else{
                        if(refer_system_msg.robotHealth[msg->data[3]-'0'] - 20 >=2000){
                            refer_system_msg.robotHealth[msg->data[3]-'0']=0;
                            //disable move and shoot
                            refer_system_msg.robotMove[msg->data[3]-'0'] = 0;
                            refer_system_msg.robotShoot[msg->data[3]-'0'] = 0;
                            ROS_INFO_STREAM("Died!");
                        }
                        else refer_system_msg.robotHealth[msg->data[3]-'0'] -= 20; // hit the head
                        ROS_INFO_STREAM("Hit the head!");
                    }
                }
                else{
                    ROS_INFO_STREAM("Miss!");
                }
            }
            else{
                ROS_INFO_STREAM("No bullet or cannot shoot debuff!");
            }
            }
        }
    void obstacleCallBack0(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg){
          filter_msg[0].circles.clear();
          filter_msg[0].segments.clear();
          filter_msg[0].header.frame_id = "map";
          for(auto it_c = obstacles_msg->circles.begin() ; it_c!=obstacles_msg->circles.end(); it_c++){
            filter_msg[0].circles.push_back(*it_c);
        }
    }
    void obstacleCallBack1(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg){
          filter_msg[1].circles.clear();
          filter_msg[1].segments.clear();
          filter_msg[1].header.frame_id = "map";
          for(auto it_c = obstacles_msg->circles.begin() ; it_c!=obstacles_msg->circles.end(); it_c++){
            filter_msg[1].circles.push_back(*it_c);
        }
    }
    void obstacleCallBack2(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg){
          filter_msg[2].circles.clear();
          filter_msg[2].segments.clear();
          filter_msg[2].header.frame_id = "map";
          for(auto it_c = obstacles_msg->circles.begin() ; it_c!=obstacles_msg->circles.end(); it_c++){
            filter_msg[2].circles.push_back(*it_c);
        }
    }
    void obstacleCallBack3(const obstacle_detector::Obstacles::ConstPtr& obstacles_msg){
          filter_msg[3].circles.clear();
          filter_msg[3].segments.clear();
          filter_msg[3].header.frame_id = "map";
          for(auto it_c = obstacles_msg->circles.begin() ; it_c!=obstacles_msg->circles.end(); it_c++){
            filter_msg[3].circles.push_back(*it_c);
        }
    }
    void cmdCallback0(const roborts_msgs::TwistAccel::ConstPtr& msg)
    {
        if(refer_system_msg.robotMove[0] == 1){
        geometry_msgs::Twist cmd;
        cmd = msg->twist;
        cmd_pub[0].publish(cmd);
        }
    }
    void cmdCallback1(const roborts_msgs::TwistAccel::ConstPtr& msg)
    {
        if(refer_system_msg.robotMove[1] == 1){
            geometry_msgs::Twist cmd;
            cmd = msg->twist;
            cmd_pub[1].publish(cmd);
        }
    }
    void cmdCallback2(const roborts_msgs::TwistAccel::ConstPtr& msg)
    {
        if(refer_system_msg.robotMove[2] == 1){
            geometry_msgs::Twist cmd;
            cmd = msg->twist;
            cmd_pub[2].publish(cmd);
        }
    }
    void cmdCallback3(const roborts_msgs::TwistAccel::ConstPtr& msg)
    {
        if(refer_system_msg.robotMove[3] == 1){
            geometry_msgs::Twist cmd;
            cmd = msg->twist;
            cmd_pub[3].publish(cmd);
        }
    }
    void game_spin(){
        try{
            for (int i = 0; i < 4; ++i) {
                listener.lookupTransform("/map", "/jackal"+std::to_string(i)+"/base_link",
                                         ros::Time(0), robot_status[i]);
            }
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        refer_system_msg.header.stamp = ros::Time::now();
        
        if(start_secs==0.) start_secs=ros::Time::now().toSec();
        
        refer_system_msg.gameTime = ros::Time::now().toSec()-start_secs;
        // ROS_INFO_STREAM("ros::Time::now().toSec() "<<ros::Time::now().toSec());
        // ROS_INFO_STREAM("start_secs "<<start_secs);

        for (int i = 0; i < 4; ++i) {
            refer_system_msg.robotPose[i].x = robot_status[i].getOrigin().x();
            refer_system_msg.robotPose[i].y = robot_status[i].getOrigin().y();
            refer_system_msg.robotPose[i].yaw = tf::getYaw(robot_status[i].getRotation());
            
            if(refer_system_msg.robotShoot[i]== 0 && refer_system_msg.robotShootDebuffTime[i]+10.<refer_system_msg.gameTime) refer_system_msg.robotShoot[i]=1;
            if(refer_system_msg.robotMove[i]== 0 && refer_system_msg.robotMoveDebuffTime[i]+10.<refer_system_msg.gameTime) refer_system_msg.robotMove[i]=1;

            // buff or debuff
            float interval=0.2;
            for (int j = 0; j<6; j++){
                if(refer_system_msg.robotPose[i].x>refer_system_msg.cellX[j]-interval && refer_system_msg.robotPose[i].x<refer_system_msg.cellX[j]+interval && refer_system_msg.robotPose[i].y>refer_system_msg.cellY[j]-interval &&refer_system_msg.robotPose[i].y<refer_system_msg.cellY[j]+interval && refer_system_msg.cellState[j].state[0]== 0 ){
                    refer_system_msg.cellState[j].state[0]= 1;
                    if( refer_system_msg.cellState[j].state[1]==1 ){
                        // can not move debuff
                        refer_system_msg.robotMove[i]= 0;
                        refer_system_msg.robotMoveDebuffTime[i]= refer_system_msg.gameTime;
                    }
                    else if (refer_system_msg.cellState[j].state[2]==1){
                        // can not shoot debuff
                        refer_system_msg.robotShoot[i]= 0;
                        refer_system_msg.robotShootDebuffTime[i]= refer_system_msg.gameTime;
                    }
                    else if( refer_system_msg.cellState[j].state[3]==1){
                        // red add health buff
                        if(refer_system_msg.robotHealth[0]+200>=2000){
                            refer_system_msg.robotHealth[0] = 2000;
                        }
                        else refer_system_msg.robotHealth[0] += 200;

                        if(refer_system_msg.robotHealth[1]+200>=2000){
                            refer_system_msg.robotHealth[1] = 2000;
                        }
                        else refer_system_msg.robotHealth[1] += 200;
                        
                    }
                    else if( refer_system_msg.cellState[j].state[4]==1){
                        // blue add health buff
                        if(refer_system_msg.robotHealth[2]+200>=2000){
                            refer_system_msg.robotHealth[2] = 2000;
                        }
                        else refer_system_msg.robotHealth[2] += 200;

                        if(refer_system_msg.robotHealth[3]+200>=2000){
                            refer_system_msg.robotHealth[3] = 2000;
                        }
                        else refer_system_msg.robotHealth[3] += 200;
                        
                    }
                    else if( refer_system_msg.cellState[j].state[5]==1){
                        // red add bullet buff
                        refer_system_msg.remain_bullet[0]+=100;
                        refer_system_msg.remain_bullet[1]+=100;
                    }
                    else if( refer_system_msg.cellState[j].state[6]==1){
                        // blue add bullet buff
                        refer_system_msg.remain_bullet[2]+=100;
                        refer_system_msg.remain_bullet[3]+=100;
                    }
            }
            }
            // ROS_INFO_STREAM("refer_system_msg.robotPose[i].y" << refer_system_msg.robotPose[i].y);
        }

        // Set cellState
        // ROS_INFO_STREAM("fabs(refer_system_msg.gameTime-int(refer_system_msg.gameTime/60)*60) "<<fabs(refer_system_msg.gameTime-int(refer_system_msg.gameTime/60)*60));
        if (fabs(refer_system_msg.gameTime-int(refer_system_msg.gameTime/60)*60)<0.009){ 
            //refresh all cell state
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 7; ++j) {
                    refer_system_msg.cellState[i].state[j]=0;
                }
            }
            std::list<int> lst;
            for(int i=1;i<7;i++){
                lst.push_back(i);
            }
            srand(time(NULL));
            int ranIndex = rand()%6+1; //1 or 2 or 3 or 4 or 5 or 6
            refer_system_msg.cellState[0].state[ranIndex]=1;
            lst.remove(ranIndex);
            if(ranIndex%2==0){
                refer_system_msg.cellState[3].state[ranIndex-1]=1;
                lst.remove(ranIndex-1);
            }
            else {
                refer_system_msg.cellState[3].state[ranIndex+1]=1;
                lst.remove(ranIndex+1);
            }
            ranIndex = rand()%4; // 0 or 1 or 2 or 3 
            std::list<int>::const_iterator iter = lst.begin();
            int i = 0;
            while(i< ranIndex) {
                iter++;
                i++;
            }
            refer_system_msg.cellState[1].state[*iter]=1;
            lst.remove(*iter);
            if(*iter%2==0){
                refer_system_msg.cellState[4].state[*iter-1]=1;
                lst.remove(*iter-1);
            }
            else {
                refer_system_msg.cellState[4].state[*iter+1]=1;
                lst.remove(*iter+1);
            }

            ranIndex = rand()%2; //0 or 1
            iter = lst.begin();
            i = 0;
            while(i< ranIndex) {
                iter++;
                i++;
            }
            refer_system_msg.cellState[2].state[*iter]=1;
            lst.remove(*iter);
            if(*iter%2==0){
                refer_system_msg.cellState[5].state[*iter-1]=1;
                lst.remove(*iter-1);
            }
            else {
                refer_system_msg.cellState[5].state[*iter+1]=1;
                lst.remove(*iter+1);
            }
            // ROS_INFO_STREAM("ranIndex" << ranIndex);            
            ROS_INFO_STREAM("refer_system_msg" << refer_system_msg);
        }
        for(int i=0 ; i<6 ; i++){
            int sum=0;
            for(int j=6; j>-1;j--){
                sum += pow(2,6-j)*refer_system_msg.cellState[i].state[j];
            }
            refer_system_msg.cellStateNumber[i]=sum;
        }

//        if(fabs(refer_system_msg.gameTime-int(refer_system_msg.gameTime/180)*180)<0.009&&int(refer_system_msg.gameTime)!=0) {
//            initialize();
//        }

        state_pub.publish(refer_system_msg);
}


private:
    ros::NodeHandle                 nh_;
    ros::Publisher                  state_pub;
    ros::Subscriber                 shoot_sub[4];
    ros::Subscriber                 obstacle[4];
    obstacle_detector::Obstacles    filter_msg[4];
    tf::TransformListener           listener;
    rm_simulator::ReferSystemInfo   refer_system_msg;
    tf::StampedTransform            robot_status[4];
    double                          start_secs=0.;
    ros::ServiceServer              game_server;
    ros::Subscriber                 acc_sub[4];
    ros::Publisher                  cmd_pub[4];
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "simulate_refer_system_node");
    ros::NodeHandle node_handle;
    
    ReferSystem referSystem(node_handle);

    // ReferSystem initialize();

    ros::Rate loop_rate(120);
    // ros::AsyncSpinner spinner(4);
    // spinner.start();

    

    while (ros::ok()){
        if(isGameStart){
            referSystem.game_spin();
//            ROS_INFO_STREAM("GAME IS PLAYING NOW!");
        }
        else{
//            ROS_INFO_STREAM("GAME IS NOT PLAYING NOW!");
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    // ros::waitForShutdown();
    return 0;
}
