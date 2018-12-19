#include <costmap_publisher_1/costmap_publisher_1.hpp>

using std::cout;
using std::endl;
costmap_publisher::costmap_publisher()
{
    subscriber.setCallbackQueue(&map_queue);
    map_sub=subscriber.subscribe("/map", 1, &costmap_publisher::map_info_setter, this);
    cost_pub=publisher.advertise<map_msgs::OccupancyGridUpdate>("/move_base/global_costmap/costmap_updates", 1);    
}
costmap_publisher::~costmap_publisher()
{}

void costmap_publisher::map_info_setter(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
//void costmap_publisher::map_info_setter(const map_msgs::OccupancyGridUpdate::ConstPtr &map_msg)
{
    cout << "[map_info_setter]" <<endl;
    map_data = *map_msg;
    map_update.header = map_msg->header;
    map_update.width = map_msg->info.width;
    map_update.height = map_msg->info.height;
    map_update.data = map_msg->data;

    cout << "map_data size :" << map_data.data.size()<< endl;
    cout << "map_msg size :" << map_msg->data.size()<< endl;
    cout << "[map_info_setter]" <<endl;
}
void costmap_publisher::map_data_publisher(void)
{
    cout << "[map_data_publisher]" <<endl;
    cost_pub.publish(map_update);
    cout << "[map_data_publisher]" <<endl;
}
void costmap_publisher::mainloop()
{
    while(ros::ok())
    {
        cout << "[mainloop start.]" <<endl;
        map_queue.callOne(ros::WallDuration(0.5));
        map_data_publisher();
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
