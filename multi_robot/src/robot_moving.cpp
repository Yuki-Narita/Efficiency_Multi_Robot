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

    //intからStrへの変換用
    std::ostringstream oss;
    std::string param_robot_str;

    ros::Rate r(1);
    std::vector<std::string> nodelist;
    bool nodelist_flag=false;
    while(!nodelist_flag)
    {
        ros::master::getNodes(nodelist);
        for(int i=0; i<nodelist.size();i++)
        {
            std::cout << "nodelit name: " << nodelist[i] << std::endl;
            if(nodelist[i].find("/multi_planning_server/server_planning"))
            {
                nodelist_flag=true;
            }
        }

    }

    //multi_planning_serverのパラメータにあるロボットの数を更新する。
    param_update.getParam("/multi_planning_server/robot_num",param_robot_num);
    param_robot_num++;
    oss << param_robot_num;
    param_robot_str = oss.str();
    param_update.setParam("/multi_planning_server/robot_num",param_robot_num);
    //ロボットからサーバーへサービスを配布する。
    std::cout << "param_robot_num: " << param_robot_num << std::endl;
    std::cout << "param_robot_str: " << param_robot_str<< std::endl;
    std::string srv_name;
    srv_name = "/robot"+param_robot_str+"/TURN";
    std::cout << "service srv_name : " << srv_name << std::endl;
    ros::ServiceServer srv = srv_nh.advertiseService(srv_name, &robot_moving::srvCB, &RM);
    ROS_INFO_STREAM("service is ready.");
    while(ros::ok() && !RM.turn_fin)
    {
        ros::spinOnce();
        std::cout << "test" << std::endl;
        r.sleep();
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
            r.sleep();
            //ifで目的地に到着したかを判定する。
        }
        else if(RM.Target_flag && RM.arrive_flag)
        {
            std::cout << "ターゲットがあってかつ到着した。" << std::endl;
            r.sleep();
        }
        else
        {
            std::cout << "問題発生。処理を終了する。" << std::endl;
            RM.stop = true;
        }
    }
    return 0;
}
