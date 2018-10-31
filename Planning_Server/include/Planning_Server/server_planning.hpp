#ifndef _SERVER_PLANNING_H_
#define _SERVER_PLANNING_H_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>


class server_planning
{
    private:
    geometry_msgs::PoseStamped TARGET;
    bool map_input;

    public:
    server_planning();
    ~server_planning();
    void highlow(void);
    void OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void getfrontier(const geometry_msgs::PoseStamped::ConstPtr &Target);
    bool map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

server_planning::server_planning()
{
    ros::Subscriber path_sub;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::CallbackQueue queueS;
    ros::CallbackQueue queueF;
    ros::Callbackqueue queueM;
    nh.setCallbackQueue(&queueS);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    path_sub=nh.subscribe("/VoronoiPlanner/path", 100, &server_planning::OptimalTarget,this);
    Target_sub=fn.subscribe("/Frontier_Target",100, &server_planning::getfrontier,this);
    map_sub=mn.subscribe("/map", 100, &server_planning::map_input, this)
}
server_planning::~server_planning()
{}
 /*
 void highlow(void)
 {}
 */  
void server_planning::getfrontier(const geometry_msgs::PoseStamped::ConstPtr &Target)
{    
       TARGET.header = Target -> header;
       TARGET.pose = Target -> pose;
}
void server_planning::OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target)
{
    
}
bool server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    nav_core::OccupancyGrid m_msg;
    m_msg.header = msg -> header;
    m_msg.info = msg -> info;
    map_input = true;
    ROS_INFO_STREAM("Mapが更新された。");
    return map_input;
}

#endif