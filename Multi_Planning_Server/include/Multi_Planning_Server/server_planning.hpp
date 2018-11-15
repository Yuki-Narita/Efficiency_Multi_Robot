#ifndef _SERVER_PLANNING_HPP_
#define _SERVER_PLANNING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/OccupancyGrid.h>
#include<std_srvs/Empty.h>
#include<std_msgs/String.h>

bool arrive_flag;
int robot_num;
int fro_num;

class server_planning
{
    private:
    geometry_msgs::PoseStamped TARGET;

    public:
    server_planning();
    ~server_planning();
    void OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void getfrontier(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void calculate_path_length(void);
    void map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    bool map_isinput(void);
    void turn_fin_CB(const std_msgs::String::ConstPtr &turn_msg);

    ros::Subscriber path_sub;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::CallbackQueue queueS;
    ros::CallbackQueue queueF;
    ros::CallbackQueue queueM;
    std_msgs::String pub_msg;
    std_msgs::String sub_msg;
    bool isinput;
    bool turn_fin;
};

server_planning::server_planning()
{
    nh.setCallbackQueue(&queueS);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    path_sub=nh.subscribe("/VoronoiPlanner/path", 100, &server_planning::OptimalTarget,this);
    Target_sub=fn.subscribe("/Frontier_Target", 100, &server_planning::getfrontier,this);
    map_sub=mn.subscribe("/map", 100, &server_planning::map_input, this);
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
    //ロボットの自己位置と各目標地点とのパスを取得してロボットの自己位置に対する最も近い
    //
}
void server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    nav_msgs::OccupancyGrid m_msg;
    m_msg.header = msg -> header;
    m_msg.info = msg -> info;
    isinput = true;
    ROS_INFO_STREAM("Map is updated");
}
bool server_planning::map_isinput(void)
{
    return isinput;
}
void calculate_path_length(void)
{

}
void server_planning::turn_fin_CB(const std_msgs::String::ConstPtr &msg)
{
    sub_msg = *msg;
    turn_fin = true;
    std::cout << "turn_fin_CB was done." << std::endl;
}


#endif