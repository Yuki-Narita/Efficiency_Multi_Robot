#ifndef _ROBOT2_ACTION_HPP_
#define _ROBOT2_ACTION_HPP_

#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>

class robot2_action
{    
    public:
        double x;
        double y;
        std::string frame_id;
        std::string move_base_node;
        ros::CallbackQueue robot2_sub_queue;
        ros::CallbackQueue robot2_pub_queue;
        ros::Subscriber robot2_sub;
        ros::Publisher robot2_pub;
        ros::NodeHandle param2;
        ros::NodeHandle robot2_pub_nh;
        ros::NodeHandle robot2_sub_nh;
        ros::Rate rate = 10;
        std_msgs::Bool arrive_flag2;
        bool wait_flag = false;
        robot2_action();
        ~robot2_action();
        void data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode);

};

#endif
        