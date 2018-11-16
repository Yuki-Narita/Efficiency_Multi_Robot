#ifndef _ROBOT_MOVING_HPP_
#define _ROBOT_MOVING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<std_msgs/String.h>
#include<std_srvs/Empty.h>

std::string robot_name;
int robot_number;

class robot_moving
{
    private:
        int robot_num;
        std::string robot_name;
        geometry_msgs::PoseStamped Target;//Frontier_Searchから受け取った目的地の座標。

    public:
        robot_moving();
        ~robot_moving();

        void firstturn(void);
        void numbering(void);
        void setgoal(const geometry_msgs::PoseStamped::ConstPtr &pose);
        void running(void);
        void arrive(const nav_msgs::Odometry::ConstPtr &odom);
        bool setflag(void);
        void resetflag(void);
        bool srvCB(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
        void firstturnCB(const std_msgs::String::ConstPtr& msg);

        ros::Subscriber sub;
        ros::Publisher pub;
        ros::CallbackQueue queueR;
        ros::NodeHandle nh;
        std_msgs::String pub_msg;
        bool arrive_flag;
        bool Target_flag;
        bool stop;
        bool turn_fin = false;
};

robot_moving::robot_moving()
{
    nh.setCallbackQueue(&queueR);
    sub=nh.subscribe("/move_base_simple/goal", 100, &robot_moving::setgoal, this);
}
robot_moving::~robot_moving()
{}
void robot_moving::firstturn(void)//ロボットが最初に２回転して地図を作る時の回る関数。
{   
        int rate = 50;
        ros::Rate r(rate);
        ros::Publisher cmd_pub;
        ros::NodeHandle cn;
        const double angular_speed = 0.5;
        const double goal_angle = 2*M_PI;
        const double angular_duration = goal_angle/angular_speed;
        ROS_INFO_STREAM("goal angular: " << goal_angle);
        int ticks = int(angular_duration * rate);

        geometry_msgs::Twist move_cmd,empty;
        move_cmd.linear.x=0.0;
        move_cmd.linear.y=0.0;
        move_cmd.linear.z=0.0;
        move_cmd.angular.x=0.0;
        move_cmd.angular.y=0.0;
        move_cmd.angular.z=0.0;
        empty = move_cmd;

        move_cmd.angular.z = angular_speed;
        cmd_pub = cn.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",5);

        int i=0;
        std::cout << "探査開始" << std::endl;
        std::cout << "初期地図を作成します。" << std::endl;
        for(i=0; i<ticks; i++)
        {
            std::cout << "i:" << i << std::endl;
            cmd_pub.publish(move_cmd);
            r.sleep();
        }
        cmd_pub.publish(empty);
}
/*
void robot_moving::numbering(void)//ロボットにIDをナンバリングする関数。
{}
*/
void robot_moving::setgoal(const geometry_msgs::PoseStamped::ConstPtr &pose)//Frontier_Searchノードからパブリッシュされる目的地の座標を受け取る関数。
{
    Target.header = pose -> header;
    Target.pose = pose -> pose;    
    Target_flag = true;
}
/*
void robot_moving::running(void)
{
    //ロボットの探査動作を書く。
}
*/
void robot_moving::arrive(const nav_msgs::Odometry::ConstPtr &odom)//ロボットが目的地に到着したかどうかを判定する関数。
{
    nav_msgs::Odometry Odom;
    Odom = *odom;
    double goal_margin = 0.5;
    float goal_dis;

    goal_dis = sqrt(pow(Target.pose.position.x,2) - pow(Odom.pose.pose.position.x,2));
    if(goal_dis >= goal_margin)
    {
        arrive_flag = false;
    }
    else
    {
        arrive_flag = true;
    }
    
}

void robot_moving::resetflag(void)
{
    Target_flag = false;
    arrive_flag = false;
}


bool robot_moving::srvCB(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res)
{
    //ROS_INFO_STREAM("robot_moving:  回転開始");
    ROS_INFO_STREAM("robot_moving:  rotate-start");
    firstturn();//最初の局所地図を作成する。
    //ROS_INFO_STREAM("robot_moving:  回転完了");
    ROS_INFO_STREAM("robot_moving:  rotate-finish");
    turn_fin = true;
    return true;
}

void robot_moving::firstturnCB(const std_msgs::String::ConstPtr& msg)
{
    std::cout << msg << std::endl;
    ROS_INFO_STREAM("robot_moving:  turning start.");
    firstturn();//最初の局所地図を作成する。
    ROS_INFO_STREAM("robot_moving:  turning complete");
    pub_msg.data = "turning done.";
    turn_fin = true;
}


#endif