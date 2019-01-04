#ifndef _COSTMAP_PUBLISHER_
#define _COSTMAP_PUBLISHER_

#include<ros/ros.h>
#include<nav_msgs/OccupancyGrid.h>
#include<ros/callback_queue.h>
#include<vector>

class costmap_publisher
{
    private:
        ros::NodeHandle robot1_publisher;
        ros::NodeHandle robot1_subscriber;
        ros::CallbackQueue robot1_map_queue;
        ros::NodeHandle robot2_publisher;
        ros::NodeHandle robot2_subscriber;
        ros::CallbackQueue robot2_map_queue;
        ros::Rate rate = 2;

        nav_msgs::OccupancyGrid map_data;
        void cost_set(void);
    
    public:
        ros::Publisher robot1_cost_pub;
        ros::Subscriber robot1_map_sub;
        ros::Publisher robot2_cost_pub;
        ros::Subscriber robot2_map_sub;
        costmap_publisher();
        ~costmap_publisher();
        void mainloop(void);
        void map_info_setter(const nav_msgs::OccupancyGrid::ConstPtr &map_msg);
        void robot1_map_data_publisher(void);
        void robot2_map_data_publisher(void);

};

#endif