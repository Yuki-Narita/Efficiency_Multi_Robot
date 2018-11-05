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
                if(arrive_flag)
                {
                    if(robot_num>fro_num)
                    {//ロボットの数の方がフロンティアセルより多いのでロボットの振り分けをする。
    
                        SP.getrobotstatus();//ロボットの状態を取得
                        SP.OptimalTarget();//状態を取得したロボットの自己位置を始点に実経路で最も近い目的地を設定する。
                        SP.setrobotstatus();//ロボットの状態を更新する。
                    }
                    else if(robot_num == fro_num)
                    {//それぞれがそれぞれの目標に向かう
                        SP.getrobotstatus();//ロボットの状態を取得
                        SP.OptimalTarget();//状態を取得したロボットの自己位置を始点に実経路で最も近い目的地を設定する。
                        SP.setrobotstatus();//ロボットの状態を更新する。
                    }
                    else if(robot_num < fro_num)
                    {
                        //
                        SP.getrobotstatus();//ロボットの状態を取得
                        SP.OptimalTarget();//状態を取得したロボットの自己位置を始点に実経路で最も近い目的地を設定する。
                        SP.setrobotstatus();//ロボットの状態を更新する。
                    }
                }
            }
            else(!SP.map_isinput())
            {
                ROS_FATAL_STREAM("Planning_Serverの起動に失敗。");
            }
        }

    
    return 0;
}
