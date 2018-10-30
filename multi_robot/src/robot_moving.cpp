#include <multi_robot/robot_moving.hpp>

int main(int argc, char const *argv[])
{
    ros::init(argc, argv, "robot_moving");
    robot_moving RM;
    ros::NodeHandle nh;
    RM.firstturn();//最初の局所地図を作成する。
    //サーバーから目的地の情報が来るまで待機。
    while(ros::ok())
    {
        ros::Subscriber sub;
        //目的地まで移動する。
        
        //ifで目的地に到着したかを判定する。
    }
    return 0;
}
