#include <multi_robot/robot_moving.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_moving");
    robot_moving RM;
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped fro_msg;
    nh.getParam("robot_name",robot_name);
    nh.getParam("robot_number",robot_number);
    

    RM.firstturn();//最初の局所地図を作成する。
    RM.getfrontier();//サーバーから目的地の情報が来るまで待機。

    
    while(!RM.stop && ros::ok())
    {
        if(RM.Target_flag && !RM.arrive_flag)
        {
            //目的地まで移動する。
            //ifで目的地に到着したかを判定する。
        }
        else if(RM.Target_flag && RM.arrive_flag)
        {
            RM.getfrontier();
        }
        else
        {
            ROS_ERROR_STREAM("問題発生。処理を終了する。");
            RM.stop = true;
        }

    }
    
    return 0;
}
