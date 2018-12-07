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
    ros::ServiceClient firstturnClient;
    server_planning SP;

    /*
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
    bool flag =false;
    std::string tmp_name;
    std::string srv_name;
    std::cout << "test2" << std::endl;
    
    while(ros::ok() && !(count == robot_num))
    {
        std::cout << "test3" << std::endl;
        if(!flag)
        {
            std::ostringstream oss;
            oss << count+1;
            tmp_name = oss.str();
            srv_name = "/robot"+tmp_name+"/TURN";
            std::cout << "service name : " << srv_name << std::endl;
            firstturnClient = srv_nh.serviceClient<std_srvs::Empty>(srv_name);
            flag = true;
        }
        bool result = firstturnClient.call(srv);
        if(result)
        {
          ROS_INFO_STREAM("complete" << tmp_name);
          count++;
          flag = false;
          tmp_name = "";
          ROS_INFO_STREAM("initialized : [" << tmp_name << "]");
        }
        else
        {
            ROS_INFO_STREAM("false");
            sleep(1);
        }

    }
    */

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
   cout << "************SERVER_PLANINNG START************" << endl;
    while(!SP.odom_queue_flag)
    {
        SP.robot1_odom_queue.callOne(ros::WallDuration(1));
    }   
    SP.odom_queue_flag=false;
    while(!SP.r1_voronoi_map_update)
    {
        cout << "r1_voronoi_map_update waiting" << endl;
        SP.r1_voronoi_map_queue.callOne(ros::WallDuration(1));
    }
    while(!SP.odom_queue_flag)
    {
        SP.robot2_odom_queue.callOne(ros::WallDuration(1));
    }
    while(!SP.r2_voronoi_map_update)
    {
        cout << "r2_voronoi_map_update waiting " << endl;
        SP.r2_voronoi_map_queue.callOne(ros::WallDuration(1));
    }
    
    //メインループ
    while(ros::ok())
    {
        SP.queueM.callOne(ros::WallDuration(1));
        if(SP.map_isinput())
        {
            //Frontier_Searchからの座標を取得
            while(!SP.queueF_judge)
            {
                SP.queueF.callOne(ros::WallDuration(1));
            }
            SP.queueF_judge = false;
            std::cout << "queueF.callOne was done." << std::endl;
            //マップとボロノイ図を比較してボロノイ経路上の目的地を絞り込む
            cout << "r1_voronoi_map_update:" << SP.r1_voronoi_map_update << endl;
            cout << "r2_voronoi_map_update:" << SP.r2_voronoi_map_update << endl;
            if(SP.r1_voronoi_map_update && SP.r2_voronoi_map_update)
            {
                cout << "r1 and r2 voronoi_map_update" << endl;
                SP.Extraction_Target();
                SP.Publish_marker();
                SP.FT2robots();//取得したフロンティア領域を各ロボットの目的地として配布。
                std::cout << "Extraction_Target was done." << std::endl;
            }
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
        cout << "loop ended" << endl;
        cout << "\n" << endl;
    }
    cout << "************SERVER_PLANINNG END************" << endl;
    return 0;
}
