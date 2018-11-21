#include <multi_robot/robot_moving.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_moving");
    int param_robot_num;
    robot_moving RM;
    ros::NodeHandle nh;
    ros::NodeHandle turn_nh;
    ros::NodeHandle param_update;

    ros::SubscribeOptions turn_req_option;
    ros::Subscriber turn_req_sub;
    ros::Publisher turn_fin_pub;

    ros::NodeHandle srv_nh;
    geometry_msgs::PoseStamped fro_msg;

    //multi_planning_serverのパラメータにあるロボットの数を更新する。
    param_update.getParam("/multi_planning_server/robot_num",param_robot_num);
    param_robot_num++;
    param_update.setParam("/multi_planning_server/robot_num",param_robot_num);
    
    //ロボットからサーバーへサービスを配布する。
    ros::ServiceServer srv = srv_nh.advertiseService("/TURN", &robot_moving::srvCB, &RM);
    ROS_INFO_STREAM("service is ready.");
    while(ros::ok() && !RM.turn_fin)
    {
        ros::spinOnce();
        std::cout << "test" << std::endl;
    }
/*  
    turn_req_sub = turn_nh.subscribe("/firstturn", 1, &robot_moving::firstturnCB, &RM);
    while(ros::ok() && !RM.turn_fin)
    {
        turn_fin_pub = turn_nh.advertise<std_msgs::String>("/turnfin", 1);
        turn_fin_pub.publish(RM.pub_msg);
        ros::spinOnce();
    }
*/
    RM.queueR.callOne(ros::WallDuration(1));//ゴールを取得
    while(!RM.stop && ros::ok())
    {
        if(RM.Target_flag && !RM.arrive_flag)
        {
            //目的地まで移動する。            
            std::cout << "ターゲットがあってかつ到着していない。" << std::endl;
            //ifで目的地に到着したかを判定する。
        }
        else if(RM.Target_flag && RM.arrive_flag)
        {
            std::cout << "ターゲットがあってかつ到着した。" << std::endl;
        }
        else
        {
            std::cout << "問題発生。処理を終了する。" << std::endl;
            RM.stop = true;
        }
    }
    return 0;
}
