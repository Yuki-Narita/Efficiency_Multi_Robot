#include<multi_planning_server/server_planning.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_planning");
    ros::Publisher pub2FS;
    ros::Subscriber subFS;
    
    //firstturn用
    ros::NodeHandle turn_nh;
    ros::Publisher turn_req_pub;
    ros::Subscriber turn_sub;

    //パラメータサーバー用
    ros::NodeHandle robot_num_nh;
    
    server_planning SP;


    //serverとつながっているロボットの個数とロボットの名前を取得
    /*
    ros::NodeHandle get_param_nh("~");
    std::vector<char>  robot_name;
    while(ros::ok())
    {
        //get_param_nh.getParam("robot_name", robot_name);//ここでパラメータの名前のところの数字をループで繰り上げる処理をして各ロボットのパラメータにアクセスしたい。
    }
    */
   //ロボットの個数をパラメータサーバーから取得
    robot_num_nh.getParam("/multi_planning_server/robot_num",robot_num);
    robot_num_nh.getParam("/multi_planning_server/given_robot_num",given_robot_num);

    // robot_movingからの初期回転の完了を検知する。
    ros::NodeHandle srv_nh;
    std_srvs::Empty srv;
    std::cout << "test1" << std::endl;
    while(ros::ok() && !(robot_num == given_robot_num))
    {
        robot_num_nh.getParam("/multi_planning_server/robot_num",robot_num);
        std::cout << "robot_num: " << robot_num << " given_robot_num: "<< given_robot_num<< std::endl;
        sleep(1);
    }
    int count=0;
    std::ostringstream oss;
    std::string tmp_name;
    std::string srv_name;
    std::cout << "test2" << std::endl;
    while(ros::ok() && !(count == robot_num))
    {
        std::cout << "test3" << std::endl;
        oss << count+1;
        tmp_name = oss.str().c_str();
        srv_name = "/robot"+tmp_name+"/TURN";
        ros::ServiceClient firstturnClient = srv_nh.serviceClient<std_srvs::Empty>(srv_name);
        bool result = firstturnClient.call(srv);
        if(result)
        {
          ROS_INFO_STREAM("complete");
          count++;
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
            std::cout << "queueF.callOne was done." << std::endl;
            //ロボットのオドメトリを取得
            SP.queueO.callOne(ros::WallDuration(1));
            std::cout << "queueO.callOne was done." << std::endl;
            //マップとボロノイ図を比較してボロノイ経路上の目的地を絞り込む
            SP.Extraction_Target();
            std::cout << "Extraction_Target was done." << std::endl;
            //それぞれのロボットの自己位置とゴールを結ぶパスを取得する。
            //
            if(robot_num >= fro_num)
            {
                //ロボットの数の方がフロンティアセルより多いのでロボットの振り分けをする。
                //SP.robot_sort1();
                std::cout << "check if 2" << std::endl;
            }
            else if(robot_num < fro_num)
            {
                //ロボットの数の方がフロンティアセルより少ない時でのロボットの振り分け方。
                //SP.robot_sort2();
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
            return 0;
        }
        SP.SP_Memory_release();
    }
    return 0;
}
