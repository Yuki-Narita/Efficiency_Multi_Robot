#ifndef _ROBOT1_ACTION_HPP_
#define _ROBOT1_ACTION_HPP_

#include<ros/ros.h>
#include<move_base_msgs/MoveBaseAction.h>
#include<actionlib/client/simple_action_client.h>

class robot1_action
{
    public:
        double x;
        double y;
        std::string frame_id;
        std::string move_base_node;
        ros::CallbackQueue robot1_queue;
        ros::Subscriber robot1_sub;
        robot1_action();
        ~robot1_action();
        void data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode);

};

#endif
        