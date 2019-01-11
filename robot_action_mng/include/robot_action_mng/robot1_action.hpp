#ifndef _ROBOT1_ACTION_HPP_
#define _ROBOT1_ACTION_HPP_

#include<ros/ros.h>
#include<std_msgs/Int8.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>
#include<nav_msgs/Odometry.h>

#include<visualization_msgs/Marker.h>

class robot1_action
{
    public:
        double x;
        double y;
        std::string frame_id;
        std::string move_base_node;
        ros::CallbackQueue robot1_pub_queue;
        ros::CallbackQueue robot1_sub_queue;
        ros::CallbackQueue robot1_odom_queue;
        ros::Subscriber robot1_sub;
        ros::Subscriber robot1_odom_sub;
        ros::Publisher robot1_pub;
        ros::NodeHandle param1;
        ros::NodeHandle robot1_pub_nh;
        ros::NodeHandle robot1_sub_nh;
        ros::NodeHandle robot1_odom_sub_nh;
        ros::Rate rate = 10;
        std_msgs::Int8 arrive_flag1;
        actionlib::SimpleClientGoalState::StateEnum goalstate;
        bool wait_flag=false;
        bool check_robot_stack=false;

        ros::NodeHandle robot1GoalNh;
        ros::Publisher robot1GoalPub;

        robot1_action();
        ~robot1_action();
        void data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode);
        void escape_robot_stack(const nav_msgs::Odometry::ConstPtr &odom);
        
        void setGoalMarker(const double x,const double y,const std::string frameId);

};

#endif
        