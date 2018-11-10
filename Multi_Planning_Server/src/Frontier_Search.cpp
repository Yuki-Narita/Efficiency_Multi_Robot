#include <Multi_Planning_Server/Frontier_Search.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Frontier_Search");

    Frontier_Search FS;

    while(ros::ok())
    {
        FS.queueF.callOne(ros::WallDuration(1));

        if(FS.isinput())
        {
            //ここにＦｒｏｎｔｉｅｒ＿Ｓｅａｒｃｈの一連の処理に必要な関数を書く
            FS.Storage();
            FS.Map_Init(FS.msg);//ここで配列にマップデータを格納する。
            FS.Side_Search();
            FS.Vatical_Search();
            FS.Side_Continuity_Search();
            FS.Vatical_Continuity_Search();
            FS.Add_Obstacle();
            FS.Search_Obstacle();
            FS.Publish_Data();
            FS.Publish_marker();
        }
        else
        {
            if(!FS.isinput())
            {
                std::cout << "not input isinput" << std::endl; 
            }
        }
        FS.resetFlag();
    }

    return 0;
}