#ifndef _ROBOT_MOVING_H_
#define _ROBOT_MOVING_H_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseStamped.h>


class robot_moving
{
    private:


    public:
        robot_moving();
        ~robot_moving();

        void firstturn(void);
        void numbering(void);
        void getfrontier(const geometry_msgs::PoseStamped::ConstPtr &pose);
};
inline geometry_msgs::PoseStamped Target;

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
void robot_moving::firstturn(void)
{   
        int rate = 50;
        ros::Rate r(rate);
        ros::Publisher cmd_pub;
        ros::NodeHandle cn;
        double angular_speed = 1.0;
        double goal_angle = 4*M_PI;
        double angular_duration = goal_angle/angular_speed;
        ROS_INFO_STREAM("goal angular: " << goal_angle);
        int ticks = int(angular_duration * rate);

        geometry_msgs::Twist move_cmd,empty;
        move_cmd.linear.x;
        move_cmd.linear.y;
        move_cmd.linear.z;
        move_cmd.angular.x;
        move_cmd.angular.y;
        move_cmd.angular.z;
        empty = move_cmd;

        move_cmd.angular.z = angular_speed;
        cmd_pub = cn.advertise<geometry_msgs::Twist>("cmd_vel",5);

        int i=0;

        while(1)
        {
            cmd_pub.publish(move_cmd);
            i++;
            if(i==ticks)break;
        }
        cmd_pub.publish(empty);
}
void robot_moving::numbering(void)
{
    
}
void robot_moving::getfrontier(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    Target.header = pose -> header;
    Target.pose = pose -> pose;
    
}

#endif