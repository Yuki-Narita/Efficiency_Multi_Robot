#include<multi_robot/first_turn_cmd.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "first_turn_cmd");
    first_turn_cmd FTC;
    ros::NodeHandle srv_nh;


    std::cout << "param_robot_str: " << FTC.param_robot_str<< std::endl;
    FTC.srv_name = "/robot"+FTC.param_robot_str+"/TURN";
    std::cout << "service srv_name : " << FTC.srv_name << std::endl;
    ros::ServiceServer srv = srv_nh.advertiseService(FTC.srv_name, &first_turn_cmd::get_server_srv, &FTC);
    ROS_INFO_STREAM("service is ready.");

    while(ros::ok() && !FTC.turn_fin)
    {
        ros::spinOnce();
        std::cout << "test" << std::endl;
        FTC.rate.sleep();
    }

    return 0;
}
