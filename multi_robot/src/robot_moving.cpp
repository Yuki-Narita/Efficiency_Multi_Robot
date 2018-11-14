#include <multi_robot/robot_moving.hpp>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_moving");

    robot_moving RM;
    ros::NodeHandle nh;
    ros::NodeHandle srv_nh;
    geometry_msgs::PoseStamped fro_msg;
    ros::ServiceServer srv = srv_nh.advertiseService("TURN", &robot_moving::srvCB, &RM);
    while(ros::ok())
    {
        ros::spinOnce();
    }


    RM.queueR.callOne(ros::WallDuration(1));//ゴールを取得

    while(!RM.stop && ros::ok())
    {
        if(RM.Target_flag && !RM.arrive_flag)
        {
            //目的地まで移動する。
            ROS_INFO_STREAM("ターゲットがあってかつ到着していない。");
            //ifで目的地に到着したかを判定する。
        }
        else if(RM.Target_flag && RM.arrive_flag)
        {
            ROS_INFO_STREAM("ターゲットがあってかつ到着した。");
        }
        else
        {
            ROS_ERROR_STREAM("問題発生。処理を終了する。");
            RM.stop = true;
        }
        
    }
    
    return 0;
}

