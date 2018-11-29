#include <multi_planning_server/Frontier_Search.hpp>

using std::cout;
using std::endl;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Frontier_Search");

    Frontier_Search FS;
    cout << "Frontier start." << endl;
    
    while(ros::ok())
    {
        cout << "test1" << endl;
        FS.queueF.callOne(ros::WallDuration(1));
        cout << "test2" << endl;
        if(FS.isinput())
        {
            //ここにＦｒｏｎｔｉｅｒ＿Ｓｅａｒｃｈの一連の処理に必要な関数を書く
            FS.Storage();
            cout << "test3" << endl;
            FS.Map_Init(FS.msg);//ここで配列にマップデータを格納する。
            cout << "test4" << endl;
            FS.Side_Search();
            cout << "test5" << endl;
            FS.Vatical_Search();
            cout << "test6" << endl;
            FS.Side_Continuity_Search();
            cout << "test7" << endl;
            FS.Vatical_Continuity_Search();
            cout << "test8" << endl;
            FS.Add_Obstacle();
            cout << "test9" << endl;
            FS.Search_Obstacle();
            cout << "test10" << endl;
            FS.Publish_Data();
            cout << "test11" << endl;
            FS.Publish_marker();
            cout << "test12" << endl;
            FS.Memory_release();
            cout << "test13" << endl;
        }
        else
        {
            cout << "else done." << endl;
            if(!FS.isinput())
            {
                std::cout << "not input isinput" << std::endl; 
            }
        }
        cout << "reset part." << endl;
        FS.resetFlag();
    }
    cout << "frontier end." << endl;
    return 0;
}