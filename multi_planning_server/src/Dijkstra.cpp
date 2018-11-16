#include<multi_planning_server/Dijkstra.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Dijkstra");

    Dijkstra DKS;


    while(ros::ok())
    {
        DKS.queueD.callOne(WallDuration(1));
        if(DKS.isinput())
        {
            DKS.mainloop();
        }
        if(!DKS.isinput())
        {
            std::cout << "no input." << std::endl;
        }
        else
        {
            std::cout << "Error" << std::endl;
        }
        DKS.reset_Flag();
    }

    return 0;
}