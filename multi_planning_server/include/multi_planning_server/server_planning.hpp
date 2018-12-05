#ifndef _SERVER_PLANNING_HPP_
#define _SERVER_PLANNING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<std_srvs/Empty.h>
#include<std_msgs/String.h>
#include<sstream>
#include<string>

using std::cout;
using std::endl;

bool path_flag1=false;
bool path_flag1_tmp=false;
bool path_flag2=false;
bool path_flag2_tmp=false;
bool arrive_flag;//ロボットが目的地に到着したかを判定する用
int robot_num;//ロボットの個数、台数。
int fro_num;//フロンティア領域の個数。
int given_robot_num;//launchの引数として与えられたロボットの台数。
nav_msgs::OccupancyGrid map_data;//大元のmapトピックのマップ情報
uint32_t map_width;
uint32_t map_height;
int **Voronoi_grid_array;
int **Frontier_array;
float map_resolution;

class server_planning
{
    private:
    nav_msgs::Odometry robot_odom;
    std::vector<geometry_msgs::PoseStamped> TARGET;
    std::vector<float> robot1lengths;
    std::vector<float> robot2lengths;

    public:
    server_planning();
    ~server_planning();
    void OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void frontier_target_CB(const geometry_msgs::PoseArray &Target);//FSノードのターゲットの情報をTARGETに格納してここで使えるようにする。
    void frontier_target2map(const std::vector<geometry_msgs::PoseStamped>& Target);//TARGETをマップ配列に入れなおす。（ボロノイ配列と比較できるようにするために）
    void map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg);//mapの更新を監視する
    bool map_isinput(void);//mapが更新したことを返す関数
    void voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg);
    void turn_fin_CB(const std_msgs::String::ConstPtr &turn_msg);//最初にロボットを回す関数
    //void robot_sort1(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier);//ロボットの個数がフロンティアセルの個数より多い時の振り分け
    //void robot_sort2(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier);//ロボットの個数がフロンティアセルの個数より少ない時の振り分け    
    void odomCB(const nav_msgs::Odometry::ConstPtr &odom_msg);//オドメトリを取得する関数
    void Extraction_Target(void);//ボロノイグリッドと重なるフロンティア座標を抽出する関数。
    void SP_Memory_release(void);//動的に確保したメモリを開放
    void FT2robots(void);//フロンティア領域を各ロボットに送信する用の関数。
    void robot1path(const  nav_msgs::Path::ConstPtr &path_msg);
    void robot2path(const nav_msgs::Path::ConstPtr &path_msg);

    ros::Subscriber path_sub1;
    ros::Subscriber path_sub2;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber voronoi_grid_sub;
    ros::Publisher pub;
    ros::Publisher target2robot1;
    ros::Publisher target2robot2;
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::NodeHandle on;
    ros::NodeHandle t2r1;
    ros::NodeHandle t2r2;
    ros::NodeHandle voronoi_map_nh;
    ros::CallbackQueue queue1;
    ros::CallbackQueue queue2;
    ros::CallbackQueue queueF;
    ros::CallbackQueue queueM;
    ros::CallbackQueue queueO;
    ros::CallbackQueue voronoi_map_queue;
    std_msgs::String pub_msg;
    std_msgs::String sub_msg;
    
    bool isinput;
    bool turn_fin;
    bool voronoi_map_update=false;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_m;
    std::string tmp_name;
};

server_planning::server_planning()
{
    nh1.setCallbackQueue(&queue1);
    nh2.setCallbackQueue(&queue2);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    on.setCallbackQueue(&queueO);
    voronoi_map_nh.setCallbackQueue(&voronoi_map_queue);

    
    target2robot1 = t2r1.advertise<geometry_msgs::PoseStamped>("/robot1/move_base_simple/goal",100);
    target2robot2 = t2r2.advertise<geometry_msgs::PoseStamped>("/robot2/move_base_simple/goal",100);
    path_sub1=nh1.subscribe("/robot1/move_base/VoronoiPlanner/plan", 100, &server_planning::robot1path, this);
    path_sub2=nh2.subscribe("/robot2/move_base/VoronoiPlanner/plan", 100, &server_planning::robot2path, this);
    Target_sub=fn.subscribe("/Frontier_Target", 100, &server_planning::frontier_target_CB, this);
    map_sub=mn.subscribe("/server/grid_map_merge/merge_map", 100, &server_planning::map_input, this);
    odom_sub=on.subscribe("/odom", 100, &server_planning::odomCB, this);
    voronoi_grid_sub=voronoi_map_nh.subscribe("/move_base/VoronoiPlanner/grid", 100, &server_planning::voronoi_map_CB, this);
}
server_planning::~server_planning()
{}
void server_planning::odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    robot_odom = *odom_msg;
}

void server_planning::frontier_target_CB(const geometry_msgs::PoseArray &Target)
{    
    cout << "frontier_target_CB done." << endl;
    fro_num = Target.poses.size();
    TARGET.resize(Target.poses.size());
    for(int i = 0; i < Target.poses.size(); i++)
    {
        TARGET[i].header = Target.header;
        TARGET[i].pose = Target.poses[i];
    }
    frontier_target2map(TARGET);
    cout << "frontier_target_CB is done." << endl;
}
void server_planning::frontier_target2map(const std::vector<geometry_msgs::PoseStamped>& Target)
{
    cout << "frontier_target2map started." << endl;
    //Frontier_map用の配列を確保
    cout << "test1" << endl;
    cout << "map_width :"<< map_width << "map_height :" << map_height << endl;
    Frontier_array = new int*[map_width];
    cout << "test2" << endl;
    for(int p = 0; p < map_width; p++)
    {
        Frontier_array[p] = new int[map_height];
    }
    cout << "test3" << endl;
    //Targetの型をfloatからintにする（map配列に座標を変換してその中の値を参照するため）
    std::vector<int> frontier_x;
    std::vector<int> frontier_y;
    cout << "test4" << endl;
    frontier_x.resize(Target.size());
    frontier_y.resize(Target.size());
    cout << "test5" << endl;
    for(int i = 0; i < Target.size(); i++)
    {
        frontier_x[i] = (int)Target[i].pose.position.x+1;
        frontier_y[i] = (int)Target[i].pose.position.y+1;
    }
    cout << "test6" << endl;
    //frontier_mapにフロンティアの座標のあるセルに1を格納する。なかったら０を代入する。
    for(int x = 0; x < map_width; x++)
    {
        for(int y = 0; y < map_height; y++)
        {
            for(int i = 0; i < Target.size(); i++)
            {
                if(x == frontier_x[i] && y == frontier_y[i])
                {
                    Frontier_array[x][y] = 1;
                }
                else
                {
                    Frontier_array[x][y] = 0;
                }
            }
        }
    }
    cout << "frontier_target2map is done." << endl;
}
void server_planning::OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target)
{
    //各ロボットから算出したすべてのパスの組み合わせて最も小さくなる組み合わせのパスを選択する。
    //その時のフロンティア領域を調べて目的地としてパブリッシュする。

}
void server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    cout << "map_input started." << endl;
    map_data = *msg;
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_resolution = msg -> info.resolution;
    isinput = true;
    ROS_INFO_STREAM("Map is updated");
}
bool server_planning::map_isinput(void)
{
    cout << "map_isinput started." << endl;
    return isinput;
    cout << "map_isinput is done." << endl;
}
void server_planning::turn_fin_CB(const std_msgs::String::ConstPtr &msg)
{
    cout << "turn_fin_CB started" << endl;
    sub_msg = *msg;
    turn_fin = true;
    std::cout << "turn_fin_CB was done." << std::endl;
}

void server_planning::robot1path(const nav_msgs::Path::ConstPtr &path_msg)
{
    cout << "robot1path started." << endl;
    nav_msgs::Path path_tmp = *path_msg;
    float path_length;

    for(int i=0; i<TARGET.size(); i++)
    {
        for(int j=1;j<path_tmp.poses.size();j++)
        {
            path_length += sqrt(pow(path_tmp.poses[j].pose.position.x-path_tmp.poses[j-1].pose.position.x,2)+pow(path_tmp.poses[j].pose.position.y - path_tmp.poses[j-1].pose.position.y,2));
        }
        robot1lengths.push_back(path_length);
        path_flag1 = true;
        while(!path_flag1_tmp){};
        path_flag1_tmp=false;
    }
    cout << "robot1path is done." << endl;
}
void server_planning::robot2path(const nav_msgs::Path::ConstPtr &path_msg)
{
    cout << "robot2path started." << endl;
    nav_msgs::Path path_tmp = *path_msg;
    float path_length;

    for(int i=0; i<TARGET.size(); i++)
    {
        for(int j=1;j<path_tmp.poses.size();j++)
        {
            path_length += sqrt(pow(path_tmp.poses[j].pose.position.x-path_tmp.poses[j-1].pose.position.x,2)+pow(path_tmp.poses[j].pose.position.y - path_tmp.poses[j-1].pose.position.y,2));
        }
        robot2lengths.push_back(path_length);
        path_flag2 = true;
        while(!path_flag2_tmp){};
        path_flag2_tmp=false;
    }
    cout << "robot2path started." << endl;
}
void server_planning::Extraction_Target(void)
{
    cout << "Extraction_Target started." << endl;
    geometry_msgs::PoseStamped t_target;
    for(int i=0; i < map_width; i++)
    {
        for(int j=0; j < map_height; j++)
        {
            if(Voronoi_grid_array[i][j] == -128 && Frontier_array[i][j] == 1)
            {
                t_target.pose.position.x = i;
                t_target.pose.position.y = j;
                t_target.pose.orientation.x = 0;
                t_target.pose.orientation.y = 0;
                t_target.pose.orientation.z = 0;
                t_target.pose.orientation.w = 1;
                Extraction_Target_m.push_back(t_target);
                std::cout << "Extracted target." << std::endl;
            }
        }
    }
    cout << "Extraction_Target is done." << endl;
}

void server_planning::voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg)
{
    cout << "voronoi_map_CB started." << endl;
    //ボロノイグリッド格納用の配列を確保
    Voronoi_grid_array = new int*[voronoi_map_msg->info.width];
    for(int p = 0; p < voronoi_map_msg->info.width; p++)
        {
            Voronoi_grid_array[p] = new int [voronoi_map_msg->info.height];
        }
    //ボロノイグリッドを配列に格納
    for(int i = 0; i < voronoi_map_msg->info.width; i++)
    {
        for(int j = 0; j < voronoi_map_msg->info.height; j++)
        {
            Voronoi_grid_array[i][j]=voronoi_map_msg->data[voronoi_map_msg->info.width*j+i];
        }
    }
    voronoi_map_update = true;
    cout << "voronoi_map_CB is done." << endl;
}

void server_planning::SP_Memory_release(void)
{
    cout << "Memory_release started." << endl;
    if(voronoi_map_update)
    {
        for(int p = 0; p < map_width; p++)
        {
            delete[] Voronoi_grid_array[p];
        }
        voronoi_map_update = false;
        delete[] Voronoi_grid_array;
    }
    for(int p = 0; p < map_width; p++)
    {
        delete[] Frontier_array[p];
    }
    delete[] Frontier_array;
    cout << "Memory_release is done." << endl;
}

void server_planning::FT2robots(void)   
{
    cout << "FT2robots started." << endl;

    robot1lengths.resize(TARGET.size());
    robot2lengths.resize(TARGET.size());

    std::vector<geometry_msgs::PoseStamped> robot1TARGET;
    std::vector<geometry_msgs::PoseStamped> robot2TARGET;
    std::string robot1header("/robot1/map");
    std::string robot2header("/robot2/map");
    robot1TARGET.resize(TARGET.size());
    robot2TARGET.resize(TARGET.size());


    for(int i = 0; i < robot1TARGET.size(); i++)
    {
        robot1TARGET[i].header.frame_id = robot1header;
    }
    for(int i = 0; i < robot1TARGET.size(); i++)
    {
        robot2TARGET[i].header.frame_id = robot2header;
    }
    for(int i = 0; i < TARGET.size(); i++)
    {
        target2robot1.publish(TARGET[i]);
        target2robot2.publish(TARGET[i]);
        queue1.callOne(ros::WallDuration(1));
        queue2.callOne(ros::WallDuration(1));
        path_flag1 = false;
        path_flag2 = false;
        path_flag1_tmp=true; 
        path_flag2_tmp=true; 
    }
    cout << "FT2robots is done." << endl;
}



#endif