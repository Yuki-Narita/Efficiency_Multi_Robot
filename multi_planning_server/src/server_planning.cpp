#include<multi_planning_server/server_planning.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_planning");
    ros::Publisher pub2FS;
    ros::Subscriber subFS;
    ros::NodeHandle turn_nh;//firstturn用のノードハンドル。
    ros::Publisher turn_req_pub;//
    ros::Subscriber turn_sub;
    server_planning SP;

    //serverとつながっているロボットの個数とロボットの名前を取得
    ros::NodeHandle get_param_nh("~");
    std::vector<char>  robot_name;
    while(ros::ok())
    {
        get_param_nh.getParam("robot_name", robot_name);//ここでパラメータの名前のところの数字をループで繰り上げる処理をして各ロボットのパラメータにアクセスしたい。
    }
    robot_num = robot_name.size();

    // robot_movingからの初期回転の完了を検知する。
    ros::NodeHandle srv_nh;
    ros::ServiceClient firstturnClient = srv_nh.serviceClient<std_srvs::Empty>("/TURN");
    std_srvs::Empty srv;
    while(ros::ok())
    {
        std::cout << "test1" << std::endl;
        bool result = firstturnClient.call(srv);
        std::cout << "test2" << std::endl;
        if(result)
        {
          ROS_INFO_STREAM("complete");
          break;
        }
        else
        {
            ROS_INFO_STREAM("false");
        }
    }
/*
    turn_req_pub = turn_nh.advertise<std_msgs::String>("/firstturn",1);
    SP.pub_msg.data = "turn now";
    while(ros::ok() && !SP.turn_fin)
    {
        turn_sub = turn_nh.subscribe("/turnfin", 1, &server_planning::turn_fin_CB, &SP);
        turn_req_pub.publish(SP.pub_msg);
        std::cout << SP.sub_msg << std::endl;
        ros::spinOnce();
    }
*/

//メインループ
        while(ros::ok())
        {
            SP.queueM.callOne(ros::WallDuration(1));
            if(SP.map_isinput())
            {
                //Frontier_Searchからの座標を取得
                SP.queueF.callOne(ros::WallDuration(1));
                //ロボットのオドメトリを取得
                SP.queueO.callOne(ros::WallDuration(1));
                //マップとボロノイ図を比較してボロノイ経路上の目的地を絞り込む
                SP.Extraction_Target();
                std::cout << "check if" << std::endl;
                if(robot_num >= fro_num)
                {
                    //ロボットの数の方がフロンティアセルより多いのでロボットの振り分けをする。
                    SP.robot_sort1();
                    std::cout << "check if 2" << std::endl;
                }
                else if(robot_num < fro_num)
                {
                    //ロボットの数の方がフロンティアセルより少ない時でのロボットの振り分け方。
                    SP.robot_sort2();
                    std::cout << "check if 3" << std::endl;
                }
                else
                {
                    //ROS_ERROR_STREAM("フロンティアセルとロボット数の条件分岐でのエラー。");
                    ROS_ERROR_STREAM("if error");
                }
            }
            else if(!SP.map_isinput())
            {
                //ROS_FATAL_STREAM("Planning_Serveでマップの更新なし。");
                ROS_FATAL_STREAM("Planning_Serve not map-refresh");
            }
            else
            {
                //ROS_ERROR_STREAM("Planning_Serverでマップの更新に失敗。");
                ROS_ERROR_STREAM("Planning_Server miss map-refresh");
            }
        }

    
    return 0;
}
