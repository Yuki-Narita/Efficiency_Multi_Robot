#include<Multi_Planning_Server/server_planning.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server_planning");
    ros::Publisher pub2FS;
    ros::Subscriber subFS;
    server_planning SP;
    
        while(ros::ok())
        {
            SP.queueM.callOne(ros::WallDuration(1));
            if(SP.map_isinput())
            {
                if(robot_num >= fro_num)
                {
                    //ロボットの数の方がフロンティアセルより多いのでロボットの振り分けをする。
                }
                else if(robot_num < fro_num)
                {
                    //ロボットの数の方がフロンティアセルより少ない時でのロボットの振り分け方。
                }
                else
                {
                    ROS_ERROR_STREAM("フロンティアセルとロボット数の条件分岐でのエラー。");
                }
            }
            else if(!SP.map_isinput())
            {
                ROS_FATAL_STREAM("Planning_Serveでマップの更新なし。");
            }
            else
            {
                ROS_ERROR_STREAM("Planning_Serverでマップの更新に失敗。");
            }
        }

    
    return 0;
}
