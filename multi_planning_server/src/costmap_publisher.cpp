#include <multi_planning_server/costmap_publisher.hpp>

using std::cout;
using std::endl;
costmap_publisher::costmap_publisher()
{
    robot1_subscriber.setCallbackQueue(&robot1_map_queue);
    robot1_map_sub=robot1_subscriber.subscribe("/robot1/move_base/global_costmap/costmap_updates", 1, &costmap_publisher::map_info_setter, this);
    robot1_cost_pub=robot1_publisher.advertise<nav_msgs::OccupancyGrid>("/robot1/move_base/global_costmap/costmap_updates", 1);    
    robot2_subscriber.setCallbackQueue(&robot2_map_queue);
    robot2_map_sub=robot2_subscriber.subscribe("/robot2/move_base/global_costmap/costmap_updates", 1, &costmap_publisher::map_info_setter, this);
    robot2_cost_pub=robot2_publisher.advertise<nav_msgs::OccupancyGrid>("/robot2/move_base/global_costmap/costmap_updates", 1);    
}
costmap_publisher::~costmap_publisher()
{}

void costmap_publisher::map_info_setter(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
{
    cout << "[map_info_setter]" <<endl;
    map_data = *map_msg;
    cout << "map_data size :" << map_data.data.size()<< endl;
    cout << "map_msg size :" << map_msg->data.size()<< endl;
    cout << "[map_info_setter]" <<endl;
}
void costmap_publisher::robot1_map_data_publisher(void)
{
    cout << "[map_data_publisher]" <<endl;
    robot1_cost_pub.publish(map_data);
    cout << "[map_data_publisher]" <<endl;
}
void costmap_publisher::robot2_map_data_publisher(void)
{
    cout << "[map_data_publisher]" <<endl;
    robot2_cost_pub.publish(map_data);
    cout << "[map_data_publisher]" <<endl;
}
void costmap_publisher::mainloop()
{
    while(ros::ok())
    {
        cout << "[mainloop start.]" <<endl;
        robot1_map_queue.callOne(ros::WallDuration(0.5));
        robot1_map_data_publisher();
        robot2_map_queue.callOne(ros::WallDuration(0.5));
        robot2_map_data_publisher();
        rate.sleep();
        cout << "[mainloop end.]" <<endl;
    }

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "costmap_publisher");
    costmap_publisher CMTP;
    CMTP.mainloop();
    return 0;
}
