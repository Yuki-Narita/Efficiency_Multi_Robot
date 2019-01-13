#include<multi_planning_server/voronoi_updater.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voronoi_updater");
    voronoi_updater VU;

    while(ros::ok())
    {
        VU.queue1.callOne(ros::WallDuration(0.1));
        VU.queue2.callOne(ros::WallDuration(0.1));
        VU.r1_grid_pub();
        VU.r2_grid_pub();
    }
    return 0;
}
