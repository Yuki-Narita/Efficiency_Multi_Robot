#ifndef _VORONOI_UPDATER_HPP_
#define _VORONOI_UPDATER_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<nav_msgs/OccupancyGrid.h>
#include<std_msgs/Int8.h>


class voronoi_updater
{
    public:
    ros::Publisher r1_voronoi_pub;
    ros::Publisher r2_voronoi_pub;
    nav_msgs::OccupancyGrid robot1_voronoi_grid;
    nav_msgs::OccupancyGrid robot2_voronoi_grid;
    ros::NodeHandle r1_nh;
    ros::NodeHandle r2_nh;
    ros::Subscriber r1_voronoi_sub;
    ros::Subscriber r2_voronoi_sub;
    ros::CallbackQueue queue1;
    ros::CallbackQueue queue2;
    voronoi_updater();
    ~voronoi_updater();
    void r1_setter(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    void r2_setter(const nav_msgs::OccupancyGrid::ConstPtr& grid);
    void r1_grid_pub(void);
    void r2_grid_pub(void);
};

voronoi_updater::voronoi_updater()
{
    r1_nh.setCallbackQueue(&queue1);
    r2_nh.setCallbackQueue(&queue2);
    r1_voronoi_sub = r1_nh.subscribe("/robot1/move_base/VoronoiPlanner/voronoi_grid", 1, &voronoi_updater::r1_setter, this);
    r2_voronoi_sub = r2_nh.subscribe("/robot2/move_base/VoronoiPlanner/voronoi_grid", 1, &voronoi_updater::r2_setter, this);
    r1_voronoi_pub = r1_nh.advertise<nav_msgs::OccupancyGrid>("/robot1_voronoi_grid_update", 1);
    r2_voronoi_pub = r2_nh.advertise<nav_msgs::OccupancyGrid>("/robot2_voronoi_grid_update", 1);
}
voronoi_updater::~voronoi_updater(){}
void voronoi_updater::r1_setter(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
    std::cout << grid << std::endl;
    robot1_voronoi_grid = *grid;
    std::cout << robot1_voronoi_grid << std::endl;
}
void voronoi_updater::r2_setter(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
   robot2_voronoi_grid = *grid;
}
void voronoi_updater::r1_grid_pub(void)
{
    std::cout << robot1_voronoi_grid << std::endl;
    r1_voronoi_pub.publish(robot1_voronoi_grid);
}
void voronoi_updater::r2_grid_pub(void)
{
    r2_voronoi_pub.publish(robot2_voronoi_grid);
}

#endif