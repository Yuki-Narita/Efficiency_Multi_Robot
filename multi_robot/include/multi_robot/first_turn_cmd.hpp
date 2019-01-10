#ifndef _FIRST_TURN_CMD_HPP_
#define _FIRST_TURN_CMD_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<std_srvs/Empty.h>
#include<geometry_msgs/Twist.h>
#include<sstream>
#include<string>

class first_turn_cmd
{
    private:
    ros::NodeHandle wait_server_nh;
    ros::CallbackQueue queue1;
    ros::Subscriber server_req_sub;
    ros::ServiceClient firstturnClient;
    std_srvs::Empty srv;
    std::string tmp_name;
    int count = 0;
    bool server_request = false;
    bool get_server_srv_flag;

    public:
    ros::Rate rate = 1;
    std::string param_robot_str;
    std::string srv_name;
    bool turn_fin = false;

    bool first_turn(void);
    bool get_server_srv(std_srvs::Empty::Request & req, std_srvs::Empty::Response& res);
    first_turn_cmd();
    ~first_turn_cmd();
};

first_turn_cmd::first_turn_cmd(){}
first_turn_cmd::~first_turn_cmd(){}

bool first_turn_cmd::first_turn(void)
{
        int rate = 50;
        ros::Rate r(rate);
        ros::Publisher cmd_pub;
        ros::NodeHandle cn;
        const double angular_speed = 0.5;
        const double goal_angle = 3*M_PI;
        const double angular_duration = goal_angle/angular_speed;
        ROS_INFO_STREAM("goal angular: " << goal_angle);
        int ticks = int(angular_duration * rate);

        geometry_msgs::Twist move_cmd,empty;
        move_cmd.linear.x=0.0;
        move_cmd.linear.y=0.0;
        move_cmd.linear.z=0.0;
        move_cmd.angular.x=0.0;
        move_cmd.angular.y=0.0;
        move_cmd.angular.z=0.0;
        empty = move_cmd;

        move_cmd.angular.z = angular_speed;
        cmd_pub = cn.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",5);

        int i=0;
        std::cout << "探査開始" << std::endl;
        std::cout << "初期地図を作成します。" << std::endl;
        for(i=0; i<ticks; i++)
        {
            std::cout << "i:" << i << std::endl;
            cmd_pub.publish(move_cmd);
            r.sleep();
        }
        cmd_pub.publish(empty);
}
bool first_turn_cmd::get_server_srv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    //ROS_INFO_STREAM("robot_moving:  回転開始");
    ROS_INFO_STREAM("robot_moving:  rotate-start");
    first_turn();//最初の局所地図を作成する。
    //ROS_INFO_STREAM("robot_moving:  回転完了");
    ROS_INFO_STREAM("robot_moving:  rotate-finish");
    turn_fin = true;
    return true;
}



#endif