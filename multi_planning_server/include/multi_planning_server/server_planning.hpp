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
#include<visualization_msgs/Marker.h>
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
uint32_t r1_map_width;
uint32_t r1_map_height;
uint32_t r2_map_width;
uint32_t r2_map_height;
int **r1_Voronoi_grid_array;
int **r2_Voronoi_grid_array;
int **r1_enhanced_Voronoi_grid_array;
int **r2_enhanced_Voronoi_grid_array;
int **Frontier_array;
int **costmap_array;
float map_resolution;
geometry_msgs::Pose map_origin;

class server_planning
{
    private:
    nav_msgs::Odometry robot1_odom;
    nav_msgs::Odometry robot2_odom;
    std::vector<geometry_msgs::PoseStamped> TARGET;
    std::vector<float> robot1lengths;
    std::vector<float> robot2lengths;
    float robot_front_point;

    //vis用
    ros::Publisher vis_pub;
    ros::NodeHandle vis_nh;
    visualization_msgs::Marker marker;

    public:
    server_planning();
    ~server_planning();
    void OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target);
    void frontier_target_CB(const geometry_msgs::PoseArray::ConstPtr &Target);//FSノードのターゲットの情報をTARGETに格納してここで使えるようにする。
    void frontier_target2map(const std::vector<geometry_msgs::PoseStamped>& Target);//TARGETをマップ配列に入れなおす。（ボロノイ配列と比較できるようにするために）
    void map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg);//mapの更新を監視する
    bool map_isinput(void);//mapが更新したことを返す関数
    void r1_voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg);
    void r2_voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg);
    void turn_fin_CB(const std_msgs::String::ConstPtr &turn_msg);//最初にロボットを回す関数
    //void robot_sort1(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier);//ロボットの個数がフロンティアセルの個数より多い時の振り分け
    //void robot_sort2(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier);//ロボットの個数がフロンティアセルの個数より少ない時の振り分け    
    void robot1_odom_CB(const nav_msgs::Odometry::ConstPtr &odom_msg);//オドメトリを取得する関数
    void robot2_odom_CB(const nav_msgs::Odometry::ConstPtr &odom_msg);//オドメトリを取得する関数
    void costmap_CB(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg);//costmapの情報を配列に格納する。
    void Extraction_Target(void);//ボロノイグリッドと重なるフロンティア座標を抽出する関数。
    void SP_Memory_release(void);//動的に確保したメモリを開放
    void FT2robots(void);//フロンティア領域を各ロボットに送信する用の関数。
    void robot1path(const  nav_msgs::Path::ConstPtr &path_msg);
    void robot2path(const nav_msgs::Path::ConstPtr &path_msg);
    void create_robot1_grid(void);//最初にボロノイグリッドがほしいのでそれを作る用。目的地の抽出に使用する。
    void create_robot2_grid(void);//最初にボロノイグリッドがほしいのでそれを作る用。目的地の抽出に使用する。
    void enhance_voronoi_map(void);//受け取ったボロノイ図は各ロボットごとのマップの大きさしかないのでマージマップとは大きさが異なる。拡張することで比較してもセグフォを吐かないようにする。拡張している部分は情報がないので０で初期化してある。
    void Publish_marker(void);//ボロノイ図を使ってフロンティアから抽出した目的地の情報をrviz上で視覚的に確認できるようにするマーカー。

    ros::Subscriber path_sub1;
    ros::Subscriber path_sub2;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Subscriber robot1_odom_sub;
    ros::Subscriber robot2_odom_sub;
    ros::Subscriber r1_voronoi_grid_sub;
    ros::Subscriber r2_voronoi_grid_sub;
    ros::Subscriber costmap_sub;
    ros::Publisher pub;
    ros::Publisher target2robot1;
    ros::Publisher target2robot2;
    ros::Publisher test_map_pub1;
    ros::Publisher test_map_pub2;
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::NodeHandle t2r1;
    ros::NodeHandle t2r2;
    ros::NodeHandle r1_voronoi_map_nh;
    ros::NodeHandle r2_voronoi_map_nh;
    ros::NodeHandle costmap_nh;
    ros::NodeHandle robot1_odom_nh;
    ros::NodeHandle robot2_odom_nh;
    ros::NodeHandle test_map_nh1;
    ros::NodeHandle test_map_nh2;

    ros::CallbackQueue queue1;
    ros::CallbackQueue queue2;
    ros::CallbackQueue queueF;
    ros::CallbackQueue queueM;
    ros::CallbackQueue queueO;
    ros::CallbackQueue r1_voronoi_map_queue;
    ros::CallbackQueue r2_voronoi_map_queue;
    ros::CallbackQueue costmap_queue;
    ros::CallbackQueue robot1_odom_queue;
    ros::CallbackQueue robot2_odom_queue;
    std_msgs::String sub_msg;//これ多分使ってないからいらないと思う。
    
    bool isinput;
    bool turn_fin;
    bool r1_voronoi_map_update=false;
    bool r2_voronoi_map_update=false;
    bool queueF_judge=false;
    bool odom_queue_flag=false;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_r1;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_r2;
    std::string tmp_name;

    geometry_msgs::PoseStamped plot_for_robot1_vorgrid;
    geometry_msgs::PoseStamped plot_for_robot2_vorgrid;
};

server_planning::server_planning():
robot_front_point(0.5)
{
    nh1.setCallbackQueue(&queue1);
    nh2.setCallbackQueue(&queue2);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    robot1_odom_nh.setCallbackQueue(&robot1_odom_queue);
    robot2_odom_nh.setCallbackQueue(&robot2_odom_queue);
    r1_voronoi_map_nh.setCallbackQueue(&r1_voronoi_map_queue);
    r2_voronoi_map_nh.setCallbackQueue(&r2_voronoi_map_queue);
    //costmap_nh.setCallbackQueue(&costmap_queue);
    vis_pub = vis_nh.advertise<visualization_msgs::Marker>("/vis_marker/Extraction_Target", 1);
    test_map_pub1 = test_map_nh1.advertise<nav_msgs::OccupancyGrid>("/test_map1", 100);
    test_map_pub2 = test_map_nh2.advertise<nav_msgs::OccupancyGrid>("/test_map2", 100);
    target2robot1 = t2r1.advertise<geometry_msgs::PoseStamped>("/robot1/move_base_simple/goal",100);
    target2robot2 = t2r2.advertise<geometry_msgs::PoseStamped>("/robot2/move_base_simple/goal",100);
    path_sub1=nh1.subscribe("/robot1/move_base/VoronoiPlanner/plan", 100, &server_planning::robot1path, this);
    path_sub2=nh2.subscribe("/robot2/move_base/VoronoiPlanner/plan", 100, &server_planning::robot2path, this);
    Target_sub=fn.subscribe("/Frontier_Target", 100, &server_planning::frontier_target_CB, this);
    map_sub=mn.subscribe("/server/grid_map_merge/merge_map", 100, &server_planning::map_input, this);
    robot1_odom_sub=robot1_odom_nh.subscribe("/robot1/odom", 100, &server_planning::robot1_odom_CB, this);
    robot2_odom_sub=robot2_odom_nh.subscribe("/robot2/odom", 100, &server_planning::robot2_odom_CB, this);
    r1_voronoi_grid_sub=r1_voronoi_map_nh.subscribe("/robot1/move_base/VoronoiPlanner/voronoi_grid", 100, &server_planning::r1_voronoi_map_CB, this);
    r2_voronoi_grid_sub=r2_voronoi_map_nh.subscribe("/robot2/move_base/VoronoiPlanner/voronoi_grid", 100, &server_planning::r2_voronoi_map_CB, this);

}
server_planning::~server_planning()
{}
void server_planning::robot1_odom_CB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    cout << "[robot1_odom_CB]----------------------------------------" << endl;
    robot1_odom = *odom_msg;
    create_robot1_grid();
    cout << "[robot1_odom_CB]----------------------------------------\n" << endl;
}
void server_planning::robot2_odom_CB(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    cout << "[robot2_odom_CB]----------------------------------------" << endl;
    robot2_odom = *odom_msg;
    robot2_odom.pose.pose.position.x += 1.5;//ロボット２のオドメトリを世界座標系に直すため。map_merge_server.launchに世界座標系でのロボット間の座標関係が設定されているのでそこを参照。
    create_robot2_grid();
    cout << "[robot2_odom_CB]----------------------------------------\n" << endl;
}

void server_planning::frontier_target_CB(const geometry_msgs::PoseArray::ConstPtr &Target)
{    
    cout << "[frontier_target_CB]----------------------------------------" << endl;
    fro_num = Target->poses.size();
    TARGET.resize(Target->poses.size());
    cout << "TARGET size: " << TARGET.size() << endl;
    for(int i = 0; i < Target->poses.size(); i++)
    {
        TARGET[i].header = Target->header;
        TARGET[i].pose = Target->poses[i];
    }
    frontier_target2map(TARGET);
    queueF_judge = true;
    cout << "[frontier_target_CB]----------------------------------------\n" << endl;
}
void server_planning::frontier_target2map(const std::vector<geometry_msgs::PoseStamped>& Target)
{
    cout << "   [frontier_target2map]----------------------------------------" << endl;
    //Frontier_map用の配列を確保
    Frontier_array = new int*[map_width];
    for(int p = 0; p < map_width; p++)
    {
        Frontier_array[p] = new int[map_height];
    }
    
    //Targetの型をfloatからintにする（map配列に座標を変換してその中の値を参照するため）
    std::vector<int> frontier_x;
    std::vector<int> frontier_y;
    frontier_x.resize(Target.size());
    frontier_y.resize(Target.size());
    cout << "    test1 TARGET size: " << Target.size() << endl;
    for(int i = 0; i < Target.size(); i++)
    {
        frontier_x[i] = (int)Target[i].pose.position.x+1;
        frontier_y[i] = (int)Target[i].pose.position.y+1;
    }
    //frontier_mapにフロンティアの座標のあるセルに1を格納する。なかったら０を代入する。
    for(int y = 0; y < map_height; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
            for(int i = 0; i < Target.size(); i++)
            {
                if(x == frontier_x[i] && y == frontier_y[i])
                {
                    cout << "    test" << endl;
                    Frontier_array[x][y] = 1;
                }
                else
                {
                    Frontier_array[x][y] = 0;
                }
            }
        }
    }
    cout << "   [frontier_target2map]----------------------------------------\n" << endl;
}
void server_planning::OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target)
{
    //各ロボットから算出したすべてのパスの組み合わせて最も小さくなる組み合わせのパスを選択する。
    //その時のフロンティア領域を調べて目的地としてパブリッシュする。

}
void server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    cout << "[map_input]----------------------------------------" << endl;
    map_data = *msg;
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_resolution = msg -> info.resolution;
    map_origin = msg ->info.origin;
    isinput = true;
    ROS_INFO_STREAM("   Map is updated");
    cout << "   map_width :"<< map_width << "map_height :" << map_height <<  "map_resolution: " << map_resolution << endl;
    cout << "[map_input]----------------------------------------\n" << endl;
}
bool server_planning::map_isinput(void)
{
    cout << "[map_isinput]----------------------------------------" << endl;
    cout << "[map_isinput]----------------------------------------\n" << endl;
    return isinput;
}
void server_planning::turn_fin_CB(const std_msgs::String::ConstPtr &msg)
{
    cout << "[turn_fin_CB]----------------------------------------" << endl;
    sub_msg = *msg;
    turn_fin = true;
    std::cout << "[turn_fin_CB]----------------------------------------\n" << std::endl;
}

void server_planning::robot1path(const nav_msgs::Path::ConstPtr &path_msg)
{
    cout << "[robot1path]----------------------------------------" << endl;
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
    cout << "[robot1path]----------------------------------------\n" << endl;
}
void server_planning::robot2path(const nav_msgs::Path::ConstPtr &path_msg)
{
    cout << "[robot2path]----------------------------------------" << endl;
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
    cout << "[robot2path]----------------------------------------\n" << endl;
}
void server_planning::Extraction_Target(void)
{
    cout << "[Extraction_Target]----------------------------------------" << endl;    
    enhance_voronoi_map();
    cout << "test" << endl;
    geometry_msgs::PoseStamped t_target;
    cout << "test" << endl;
    for(int i=0; i < map_width; i++)
    {
        cout << "loop1" << endl;
        for(int j=0; j < map_height; j++)
        {
            if(r1_enhanced_Voronoi_grid_array[i][j] == -128 && Frontier_array[i][j] == 1)
            {
                t_target.pose.position.x = i;
                t_target.pose.position.y = j;
                t_target.pose.orientation.x = 0.0;
                t_target.pose.orientation.y = 0.0;
                t_target.pose.orientation.z = 0.0;
                t_target.pose.orientation.w = 1.0;
                Extraction_Target_r1.push_back(t_target);
                cout << "r1 Extracted target." <<  endl;
            }
        }
    }
    cout << "r1 Extracted target size: " << Extraction_Target_r1.size() << endl;
    for(int j=0; j < map_height; j++)
    {
        cout << "loop2" << endl;
        for(int i=0; i < map_width; i++)
        {
            if(r2_enhanced_Voronoi_grid_array[i][j] == -128 && Frontier_array[i][j] == 1)
            {
                t_target.pose.position.x = i;
                t_target.pose.position.y = j;
                t_target.pose.orientation.x = 0.0;
                t_target.pose.orientation.y = 0.0;
                t_target.pose.orientation.z = 0.0;
                t_target.pose.orientation.w = 1.0;
                Extraction_Target_r2.push_back(t_target);
                cout << "r2 Extracted target." << endl;
            }
        }
    }
    cout << "r2 Extracted target size: " << Extraction_Target_r2.size() << endl;
    cout << "[Extraction_Target]----------------------------------------\n" << endl;
}

void server_planning::r1_voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg)
{
    cout << "[r1_voronoi_map_CB]----------------------------------------" << endl;
    r1_map_height = voronoi_map_msg ->info.height;
    r1_map_width = voronoi_map_msg ->info.width;
    cout << "voronoi_map resolution: " << voronoi_map_msg->info.resolution << endl;
    //ボロノイグリッド格納用の配列を確保
    r1_Voronoi_grid_array = new int*[voronoi_map_msg->info.width];
    for(int p = 0; p < voronoi_map_msg->info.width; p++)
        {
            r1_Voronoi_grid_array[p] = new int [voronoi_map_msg->info.height];
        }
    //ボロノイグリッドを配列に格納
    for(int j = 0; j < voronoi_map_msg->info.height; j++)
    {
        for(int i = 0; i < voronoi_map_msg->info.width; i++)
        {
            r1_Voronoi_grid_array[i][j]=voronoi_map_msg->data[voronoi_map_msg->info.width*j+i];
        }
    }
    r1_voronoi_map_update = true;
    cout << "[r1_voronoi_map_CB]----------------------------------------\n" << endl;
}

void server_planning::r2_voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg)
{
    cout << "[r2_voronoi_map_CB]----------------------------------------" << endl;
    r2_map_height = voronoi_map_msg ->info.height;
    r2_map_width = voronoi_map_msg ->info.width;
    //ボロノイグリッド格納用の配列を確保
    r2_Voronoi_grid_array = new int*[voronoi_map_msg->info.width];
    for(int p = 0; p < voronoi_map_msg->info.width; p++)
    {
        r2_Voronoi_grid_array[p] = new int [voronoi_map_msg->info.height];
    }
    //ボロノイグリッドを配列に格納
    for(int j = 0; j < voronoi_map_msg->info.height; j++)
    {
        for(int i = 0; i < voronoi_map_msg->info.width; i++)
        {
            r2_Voronoi_grid_array[i][j]=voronoi_map_msg->data[voronoi_map_msg->info.width*j+i];
        }
    }
    r2_voronoi_map_update = true;
    cout << "[2_voronoi_map_CB]----------------------------------------\n" << endl;
}

void server_planning::SP_Memory_release(void)
{
    cout << "[Memory_release]----------------------------------------" << endl;
    cout << "r1_voronoif_map_update:" << r1_voronoi_map_update << endl;
    if(r1_voronoi_map_update != 0)
    {
        for(int p = 0; p < r1_map_width; p++)
        {
            delete[] r1_Voronoi_grid_array[p];
        }
        delete[] r1_Voronoi_grid_array;
        
        for(int p = 0; p < map_width; p++)
        {
            delete[] r1_enhanced_Voronoi_grid_array[p];
        }
        delete[] r1_enhanced_Voronoi_grid_array;
        r1_voronoi_map_update = false;
    }
    cout << "r2_vorono_map_update:" << r2_voronoi_map_update << endl;
    if(r2_voronoi_map_update != 0)
    {
        for(int p = 0; p < r2_map_width; p++)
        {
            delete[] r2_Voronoi_grid_array[p];
        }
        delete[] r2_Voronoi_grid_array;
        
        for(int p = 0; p < map_width; p++)
        {
            delete[] r2_enhanced_Voronoi_grid_array[p];
        }
        delete[] r2_Voronoi_grid_array;
        r2_voronoi_map_update = false;
    }
    for(int p = 0; p < map_width; p++)
    {
        delete[] Frontier_array[p];
    }
    delete[] Frontier_array;
    cout << "[Memory_release]----------------------------------------\n" << endl;
}

void server_planning::FT2robots(void)   
{
    cout << "[FT2robots]----------------------------------------" << endl;

    robot1lengths.resize(TARGET.size());
    robot2lengths.resize(TARGET.size());
    std::vector<geometry_msgs::PoseStamped> robot1TARGET;
    std::vector<geometry_msgs::PoseStamped> robot2TARGET;
    std::string robot1header("/robot1/map");
    std::string robot2header("/robot2/map");
    robot1TARGET.resize(TARGET.size());
    robot2TARGET.resize(TARGET.size());
    sleep(2);
    for(int i = 0; i < robot1TARGET.size(); i++)
    {
        robot1TARGET[i].header.frame_id = robot1header;
    }
    for(int i = 0; i < robot2TARGET.size(); i++)
    {
        robot2TARGET[i].header.frame_id = robot2header;
    }
    for(int i = 0; i < TARGET.size(); i++)
    {
        target2robot1.publish(TARGET[i]);
        target2robot2.publish(TARGET[i]);
        queue1.callOne(ros::WallDuration(1.5));
        queue2.callOne(ros::WallDuration(1.5));
        path_flag1 = false;
        path_flag2 = false;
        path_flag1_tmp=true; 
        path_flag2_tmp=true; 
    }
    cout << "[FT2robots]----------------------------------------\n" << endl;
}

void server_planning::costmap_CB(const nav_msgs::OccupancyGrid::ConstPtr& costmap_msg)
{
    cout << "***costmap_CB started.***" << endl;
    //コストマップ格納用の配列を確保
    costmap_array = new int*[costmap_msg->info.width];
    for(int p = 0; p < costmap_msg->info.width; p++)
        {
            costmap_array[p] = new int [costmap_msg->info.height];
        }
    //コストマップの情報を配列に格納
    for(int i = 0; i < costmap_msg->info.width; i++)
    {
        for(int j = 0; j < costmap_msg->info.height; j++)
        {
            costmap_array[i][j]=costmap_msg->data[costmap_msg->info.width*j+i];
        }
    }
    cout << "***costmap_CB is done.***\n" << endl;
}
void server_planning::create_robot1_grid(void)
{
    cout << "   [create_robot1_grid]----------------------------------------" << endl;
    plot_for_robot1_vorgrid.header.frame_id = "/server/map";
    plot_for_robot1_vorgrid.header.stamp = ros::Time::now();
    plot_for_robot1_vorgrid.pose.position.x = robot1_odom.pose.pose.position.x + robot_front_point;
    plot_for_robot1_vorgrid.pose.position.y = 0.0;
    plot_for_robot1_vorgrid.pose.position.z = 0.0;
    plot_for_robot1_vorgrid.pose.orientation.x = 0.0;
    plot_for_robot1_vorgrid.pose.orientation.y = 0.0;
    plot_for_robot1_vorgrid.pose.orientation.z = 0.0;
    plot_for_robot1_vorgrid.pose.orientation.w = 1.0;

    target2robot1.publish(plot_for_robot1_vorgrid);
    odom_queue_flag=true;
    cout << "   [create_robot1_grid]----------------------------------------\n" << endl;
}
void server_planning::create_robot2_grid(void)
{
    cout << "   [create_robot2_grid]----------------------------------------" << endl;
    plot_for_robot2_vorgrid.header.frame_id = "/server/map";
    plot_for_robot2_vorgrid.header.stamp = ros::Time::now();
    plot_for_robot2_vorgrid.pose.position.x = robot2_odom.pose.pose.position.x + robot_front_point;
    plot_for_robot2_vorgrid.pose.position.y = 0.0;
    plot_for_robot2_vorgrid.pose.position.z = 0.0;
    plot_for_robot2_vorgrid.pose.orientation.x = 0.0;
    plot_for_robot2_vorgrid.pose.orientation.y = 0.0;
    plot_for_robot2_vorgrid.pose.orientation.z = 0.0;
    plot_for_robot2_vorgrid.pose.orientation.w = 1.0;
    target2robot2.publish(plot_for_robot2_vorgrid);
    odom_queue_flag=true;
    cout << "   [create_robot2_grid]----------------------------------------\n" << endl;
}

void server_planning::enhance_voronoi_map(void)
{
    cout << "   [enhance_voronoi_map]----------------------------------------" << endl;
    nav_msgs::OccupancyGrid test_map1;
    nav_msgs::OccupancyGrid test_map2;
    test_map1.header.frame_id = "/server/map";
    test_map1.header.stamp = ros::Time::now();
    test_map1.info.resolution = map_resolution;
    test_map1.info.height = map_height;
    test_map1.info.width = map_width;
    test_map1.info.origin = map_origin;
    test_map2.header.frame_id = "/server/map";
    test_map2.header.stamp = ros::Time::now();
    test_map2.info.resolution = map_resolution;
    test_map2.info.height = map_height;
    test_map2.info.width = map_width;
    test_map2.info.origin = map_origin;

    //Frontierとの比較用に拡張させたボロノイ配列を作成する。
    r1_enhanced_Voronoi_grid_array = new int*[map_width];
    for(int p = 0; p < map_width; p++)
    {
        r1_enhanced_Voronoi_grid_array[p] = new int[map_height];
    }
    //拡張したボロノイ配列を0で初期化する。
    for(int y = 0; y < map_height; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
            r1_enhanced_Voronoi_grid_array[x][y] = 0;
        }
    }
    //拡張したボロノイ配列にトピックから受け取ったボロノイ図の情報を反映する。
    for(int y = 0; y < r1_map_height; y++)
    {
        for(int x = 0; x < r1_map_width; x++)
        {
            r1_enhanced_Voronoi_grid_array[x][y] = r1_Voronoi_grid_array[x][y];
            test_map1.data.push_back(r1_enhanced_Voronoi_grid_array[x][y]);
        }
    }
    
    cout << "   data size: " << test_map1.data.size() << endl;
    test_map_pub1.publish(test_map1);

    r2_enhanced_Voronoi_grid_array = new int*[map_width];
    for(int p = 0; p < map_width; p++)
    {
        r2_enhanced_Voronoi_grid_array[p] = new int[map_height];
    }
    //拡張したボロノイ配列を0で初期化する。
    for(int y = 0; y < map_height; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
            r2_enhanced_Voronoi_grid_array[x][y] = 0;
        }
    }
    //拡張したボロノイ配列にトピックから受け取ったボロノイ図の情報を反映する。
    for(int y = 0; y < r2_map_height; y++)
    {
        for(int x = 0; x < r2_map_width; x++)
        {
            r2_enhanced_Voronoi_grid_array[x][y] = r2_Voronoi_grid_array[x][y];
        }
    }
    for (int y = 0; y < map_height; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
            test_map2.data.push_back(r2_enhanced_Voronoi_grid_array[x][y]);
        }
    }
    test_map_pub2.publish(test_map2);

    cout << "   [enhance_voronoi_map]----------------------------------------" << endl;
}

void server_planning::Publish_marker(void)
{
    cout << "[publish_marker]----------------------------------------" << endl;
    uint32_t shape = visualization_msgs::Marker::CUBE_LIST;
        marker.header.frame_id = "/server/merge_map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Extraction_target";
        marker.id = 0;
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;
		
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

		geometry_msgs::Point p;
		for(int i=0; i < Extraction_Target_r1.size();i++)
		{
			p.x = Extraction_Target_r1[i].pose.position.x;
			p.y = Extraction_Target_r1[i].pose.position.y;
			marker.points.push_back(p);
		}
        for(int i=0; i < Extraction_Target_r2.size();i++)
		{
			p.x = Extraction_Target_r2[i].pose.position.x;
			p.y = Extraction_Target_r2[i].pose.position.y;
			marker.points.push_back(p);
		}
        cout << "marker size: " << marker.points.size() << endl;
		vis_pub.publish(marker);
        cout << "[publish_marker]----------------------------------------" << endl;
}

#endif