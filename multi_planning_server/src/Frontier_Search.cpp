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
            FS.robot1_costmap_queue.callOne(ros::WallDuration(0.1));
            FS.robot2_costmap_queue.callOne(ros::WallDuration(0.1));
            FS.Storage();
            FS.Map_Init(FS.msg);//ここで配列にマップデータを格納する。
            FS.Side_Search();
            FS.Vatical_Search();
            FS.Side_Continuity_Search();
            FS.Vatical_Continuity_Search();
            FS.Add_Obstacle();
            FS.Search_Obstacle();
            FS.Check_Target_Overlap_Costmap1(FS.robot1_costmap_data);
            FS.Check_Target_Overlap_Costmap2(FS.robot2_costmap_data);
            FS.Publish_Data();
            FS.Publish_marker();
            FS.Memory_release();
            FS.Clear_Vector();
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