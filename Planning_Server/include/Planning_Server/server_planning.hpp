#ifndef _SERVER_PLANNING_H_
#define _SERVER_PLANNING_H_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>


class server_planning
{
    private:
    geometry_msgs::PoseStamped TARGET;

    public:
    server_planning();
    ~server_planning();
    void highlow(void);
    void targetdicision(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void getfrontier(const geometry_msgs::PoseStamped::ConstPtr &Target);
};

server_planning::server_planning()
{
    ros::Subscriber path_sub;
    ros::Subscriber Target_sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle fn;
    ros::CallbackQueue queueS;
    ros::CallbackQueue queueF;
    nh.setCallbackQueue(&queueS);
    fn.setCallbackQueue(&queueF);
    path_sub=nh.subscribe("/VoronoiPlanner/path", 100, &server_planning::targetdicision);
    Target_sub=fn.subscribe("/Frontier_Target",100, &server_planning::getfrontier);
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
void server_planning::targetdicision(const geometry_msgs::PoseStamped::ConstPtr &Target)
{
    
}

#endif