#ifndef _ROBOT_MOVING_HPP_
#define _ROBOT_MOVING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>

std::string robot_name;
int robot_number;

class robot_moving
{
    private:
        int robot_num;
        std::string robot_name;


    public:
        robot_moving();
        ~robot_moving();

        void firstturn(void);
        void numbering(void);
        void getfrontier(const geometry_msgs::PoseStamped::ConstPtr &pose);
        void running(void);
        void arrive(const nav_msgs::Odometry &odom);
        
        bool arrive_flag;
};
inline geometry_msgs::PoseStamped Target;//Frontier_Searchから受け取った目的地の座標。

robot_moving::robot_moving()
{
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::CallbackQueue queueR;
    ros::NodeHandle nh;
    nh.setCallbackQueue(&queueR);
    sub=nh.subscribe("/move_base_simple/goal", 100, &robot_moving::getfrontier, this);
}
robot_moving::~robot_moving()
{}
void robot_moving::firstturn(void)//ロボットが最初に２回転して地図を作る時の回る関数。
{   
        int rate = 50;
        ros::Rate r(rate);
        ros::Publisher cmd_pub;
        ros::NodeHandle cn;
        const double angular_speed = 0.2;
        const double goal_angle = 4*M_PI;
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
        cmd_pub = cn.advertise<geometry_msgs::Twist>("cmd_vel",5);

        int i=0;
        std::cout << "探査開始" << std::endl;
        std::cout << "初期地図を作成します。" << std::endl;
        while(1)
        {
            cmd_pub.publish(move_cmd);
            i++;
            if(i==ticks)break;
        }
        cmd_pub.publish(empty);
}
/*
void robot_moving::numbering(void)//ロボットにIDをナンバリングする関数。
{}
*/
void robot_moving::getfrontier(const geometry_msgs::PoseStamped::ConstPtr &pose)//Frontier_Searchノードからパブリッシュされる目的地の座標を受け取る関数。
{
    Target.header = pose -> header;
    Target.pose = pose -> pose;    
}
/*
void robot_moving::running(void)
{
    //ロボットの探査動作を書く。
}
*/
void robot_moving::arrive(const nav_msgs::Odometry &odom)//ロボットが目的地に到着したかどうかを判定する関数。
{
    nav_msgs::Odometry Odom;
    Odom.header = odom -> header;
    Odom.pose = odom -> pose;
    Odom.twist = odom -> twist;

    double goal_margin = 0.5;
    float goal_dis;

    goal_dis = sqrt(pow(Target.pose.position.x,2) - pow(Odom.pose.pose.position.x,2));
    if(goal_dis >= goal_margin)
    {
        arrive = false;
    }
    else
    {
        arrive = true;
    }
    
}


#endif