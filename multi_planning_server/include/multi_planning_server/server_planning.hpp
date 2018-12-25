#ifndef _SERVER_PLANNING_HPP_
#define _SERVER_PLANNING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<std_msgs/Bool.h>
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
    geometry_msgs::PoseStamped stop_pose;
    std::vector<geometry_msgs::PoseStamped> TARGET;//FSノードから取得したフロンティア領域
    std::vector<std::tuple<int, float, float, float>> robot1lengths;//ロボット１の自己位置から目的地までの長さのコンテナ
    std::vector<std::tuple<int, float, float, float>> robot2lengths;//ロボット２の自己位置から目的地までの長さのコンテナ
    int robot1path_count=0;
    int robot2path_count=0;
    std::vector<int> frontier_x;
    std::vector<int> frontier_y;
    std::vector<geometry_msgs::PoseStamped> robot1TARGET;//抽出後のロボット１への目的地
    std::vector<geometry_msgs::PoseStamped> robot2TARGET;//抽出後のロボット２への目的地
    ros::Rate rate=10;

    //初期にロボットのvorrnoi_gridを生成するために目的地として与える点。
    float robot_front_point;

    //拡張したvoronoi_mapとロボットのローカルのvoronoi_mapを一致させるためにずらす調整用の変数。
    float robot1_init_x;
    float robot1_init_y;
    float robot2_init_x;
    float robot2_init_y;
    
    //frontierを中心とした近くのvorgridを探索する用。
    float search_length;
    int search_length_cell;
    int half_sq;
    int half_leftx;//四角形の左半分の長さ
    int half_rightx;//四角形の右半分の長さ
    int half_topy;//四角形の上半分の長さ
    int half_bottomy;//四角形の下半分の長さ
    int r1_frontier_sum;//フラグが続いているかの判定用。
    int r2_frontier_sum;//フラグが続いているかの判定用。
    std::vector<int> pre_frox;
    std::vector<int> pre_froy;
    int Extracted_sum;//抽出後の目的地の数。

    //vis用
    ros::Publisher vis_pub;
    ros::NodeHandle vis_nh;


    public:
    server_planning();
    ~server_planning();
    void OptimalTarget(void);
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
    void Clear_Vector(void);
    void Clear_Num(void);//カウント初期化用
    void arrive1_flag(const std_msgs::Bool::ConstPtr &msg);
    void arrive2_flag(const std_msgs::Bool::ConstPtr &msg);

    ros::Subscriber path_sub1;
    ros::Subscriber path_sub2;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Subscriber costmap_sub;
    ros::Subscriber arrive1_sub;
    ros::Subscriber arrive2_sub;
    ros::Publisher pub;
    ros::Publisher target2robot1;
    ros::Publisher target2robot2;
    ros::Publisher test_map_pub1;
    ros::Publisher test_map_pub2;
    ros::Publisher robot1_final_target_pub;
    ros::Publisher robot2_final_target_pub;
    ros::NodeHandle nh1;
    ros::NodeHandle nh2;
    ros::NodeHandle robot1_final_target_nh;
    ros::NodeHandle robot2_final_target_nh;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::NodeHandle t2r1;
    ros::NodeHandle t2r2;
    ros::NodeHandle costmap_nh;
    ros::NodeHandle test_map_nh1;
    ros::NodeHandle test_map_nh2;
    ros::NodeHandle get_param_nh;
    ros::NodeHandle arrive1_nh;
    ros::NodeHandle arrive2_nh;
    ros::CallbackQueue queue1;
    ros::CallbackQueue queue2;
    ros::CallbackQueue queueF;
    ros::CallbackQueue queueM;
    ros::CallbackQueue queueO;
    ros::CallbackQueue costmap_queue;
    ros::CallbackQueue robot1_testmap_queue;
    ros::CallbackQueue robot2_testmap_queue;
    ros::CallbackQueue arrive1_queue;
    ros::CallbackQueue arrive2_queue;

    //voronoiマップの取得用。
    ros::NodeHandle r1_voronoi_map_nh;
    ros::Subscriber r1_voronoi_grid_sub;
    ros::CallbackQueue r1_voronoi_map_queue;
    ros::NodeHandle r2_voronoi_map_nh;
    ros::Subscriber r2_voronoi_grid_sub;
    ros::CallbackQueue r2_voronoi_map_queue;

    //ロボットのオドメトリ取得用。
    ros::NodeHandle robot1_odom_nh;
    ros::Subscriber robot1_odom_sub;
    ros::CallbackQueue robot1_odom_queue;
    ros::NodeHandle robot2_odom_nh;
    ros::Subscriber robot2_odom_sub;
    ros::CallbackQueue robot2_odom_queue;
    std_msgs::String sub_msg;//これ多分使ってないからいらないと思う。
    
    //その他
    bool isinput;
    bool turn_fin;
    bool r1_voronoi_map_update=false;
    bool r2_voronoi_map_update=false;
    bool queueF_judge=false;
    bool odom_queue_flag=false;
    bool arrive1;
    bool arrive2;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_r1;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_r2;
    std::string tmp_name;
    geometry_msgs::PoseStamped plot_for_robot1_vorgrid;
    geometry_msgs::PoseStamped plot_for_robot2_vorgrid;
};

server_planning::server_planning():
robot_front_point(0.5),
search_length(0.1)
{
    nh1.setCallbackQueue(&queue1);
    nh2.setCallbackQueue(&queue2);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    robot1_odom_nh.setCallbackQueue(&robot1_odom_queue);
    robot2_odom_nh.setCallbackQueue(&robot2_odom_queue);
    r1_voronoi_map_nh.setCallbackQueue(&r1_voronoi_map_queue);
    r2_voronoi_map_nh.setCallbackQueue(&r2_voronoi_map_queue);
    test_map_nh1.setCallbackQueue(&robot1_testmap_queue);
    test_map_nh2.setCallbackQueue(&robot2_testmap_queue);
    arrive1_nh.setCallbackQueue(&arrive1_queue);
    arrive2_nh.setCallbackQueue(&arrive2_queue);

    //costmap_nh.setCallbackQueue(&costmap_queue);
    vis_pub = vis_nh.advertise<visualization_msgs::Marker>("/vis_marker/Extraction_Target", 1);
    test_map_pub1 = test_map_nh1.advertise<nav_msgs::OccupancyGrid>("/test_map1", 1);
    test_map_pub2 = test_map_nh2.advertise<nav_msgs::OccupancyGrid>("/test_map2", 1);
    target2robot1 = t2r1.advertise<geometry_msgs::PoseStamped>("/robot1/move_base_simple/goal",1);
    target2robot2 = t2r2.advertise<geometry_msgs::PoseStamped>("/robot2/move_base_simple/goal",1);
    robot1_final_target_pub = robot1_final_target_nh.advertise<geometry_msgs::PoseStamped>("/robot1/final_target",1);
    robot2_final_target_pub = robot2_final_target_nh.advertise<geometry_msgs::PoseStamped>("/robot2/final_target",1);
    path_sub1=nh1.subscribe("/robot1/move_base/VoronoiPlanner/plan", 1000, &server_planning::robot1path, this);
    path_sub2=nh2.subscribe("/robot2/move_base/VoronoiPlanner/plan", 1000, &server_planning::robot2path, this);
    Target_sub=fn.subscribe("/Frontier_Target", 1, &server_planning::frontier_target_CB, this);
    map_sub=mn.subscribe("/server/grid_map_merge/merge_map", 1, &server_planning::map_input, this);
    robot1_odom_sub=robot1_odom_nh.subscribe("/robot1/odom", 1, &server_planning::robot1_odom_CB, this);
    robot2_odom_sub=robot2_odom_nh.subscribe("/robot2/odom", 1, &server_planning::robot2_odom_CB, this);
    r1_voronoi_grid_sub=r1_voronoi_map_nh.subscribe("/robot1/move_base/VoronoiPlanner/voronoi_grid", 1, &server_planning::r1_voronoi_map_CB, this);
    r2_voronoi_grid_sub=r2_voronoi_map_nh.subscribe("/robot2/move_base/VoronoiPlanner/voronoi_grid", 1, &server_planning::r2_voronoi_map_CB, this);
    arrive1_sub = arrive1_nh.subscribe("/multi_planning_server/robot1_action/robot1/arrive_flag", 1, &server_planning::arrive1_flag, this);
    arrive2_sub = arrive2_nh.subscribe("/multi_planning_server/robot2_action/robot2/arrive_flag", 1, &server_planning::arrive2_flag, this);
    get_param_nh.getParam("/robot1_init_x",robot1_init_x);
    get_param_nh.getParam("/robot1_init_y",robot1_init_y);
    get_param_nh.getParam("/robot2_init_x",robot2_init_x);
    get_param_nh.getParam("/robot2_init_y",robot2_init_y);
    stop_pose.pose.position.x = 0.0;
    stop_pose.pose.position.y = 0.0;
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
    create_robot2_grid();
    cout << "[robot2_odom_CB]----------------------------------------\n" << endl;
}

void server_planning::frontier_target_CB(const geometry_msgs::PoseArray::ConstPtr &Target)
{    
    cout << "[frontier_target_CB]----------------------------------------" << endl;
    TARGET.clear();
    cout << "Frontier seq: " << Target->header.seq;
    cout << "Target size: " << Target->poses.size() << endl;
    fro_num = Target->poses.size();
    TARGET.resize(fro_num);
    cout << "TARGET size: " << TARGET.size() << endl;
    for(int i = 0; i < fro_num; i++)
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
    frontier_x.resize(Target.size());
    frontier_y.resize(Target.size());
    cout << "   TARGET size: " << Target.size() << endl;
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
                    Frontier_array[x][y] = 1;
                }
                else
                {
                    Frontier_array[x][y] = 0;
                }
            }
        }
    }
    cout << "   [frontier_target2map]----------------------------------------" << endl;
}
void server_planning::OptimalTarget(void)
{
    cout << "[OptimalTarget]----------------------------------------" << endl;
    //各ロボットから算出したすべてのパスの組み合わせて最も小さくなる組み合わせのパスを選択する。
    //その時のフロンティア領域を調べて目的地としてパブリッシュする。
    float tmp_length;
    int min_length=100000;
    int seq1;
    int seq2;
    geometry_msgs::PoseStamped final_target1;
    geometry_msgs::PoseStamped final_target2;
    std::string robot1header("/robot1/map");
    std::string robot2header("/robot2/map");

    cout << "robot1lengths size:" << robot1lengths.size() << endl;
    cout << "robot2lengths size:" << robot2lengths.size() << endl;
    robot1lengths.shrink_to_fit();
    robot2lengths.shrink_to_fit();
    cout << "robot1lengths size:" << robot1lengths.size() << endl;
    cout << "robot2lengths size:" << robot2lengths.size() << endl;
    for(int i=0; i<robot1lengths.size(); i++)
    {
        for(int j=0; j<robot2lengths.size(); j++)
        {
                cout << "robot1lengths[" << i << "]: " << std::get<1>(robot1lengths[i]) << endl;
                cout << "robot2lengths[" << j << "]: " << std::get<1>(robot2lengths[j]) << endl;
            if(min_length >= std::get<1>(robot1lengths[i]) + std::get<1>(robot2lengths[i]) && 0 != std::get<1>(robot1lengths[i]) && 0 != std::get<1>(robot2lengths[i]))
            {
                min_length = std::get<1>(robot1lengths[i]) + std::get<1>(robot2lengths[i]);
                final_target1.pose.position.x = std::get<2>(robot1lengths[i]);
                final_target1.pose.position.y = std::get<3>(robot1lengths[i]);
                cout << "robot1lengths frontier_x: " << std::get<2>(robot1lengths[i]) << endl;
                cout << "robot1lengths frontier_y: " << std::get<3>(robot1lengths[i]) << endl;
                final_target2.pose.position.x = std::get<2>(robot2lengths[i]);
                final_target2.pose.position.y = std::get<3>(robot2lengths[i]);
                cout << "robot2lengths frontier_x: " << std::get<2>(robot2lengths[i]) << endl;
                cout << "robot2lengths frontier_y: " << std::get<3>(robot2lengths[i]) << endl;
                cout << "test" << endl; 
            }
        }
    }
    cout << "min_length: " << min_length << endl;
    final_target1.header.frame_id = robot1header;
    final_target2.header.frame_id = robot2header;
    final_target1.pose.orientation.w = 0.1;
    final_target2.pose.orientation.w = 0.1;
    cout << "final_target1:" << final_target1 << endl;
    cout << "final_target2:" << final_target2 << endl;
    robot1_final_target_pub.publish(final_target1);
    robot2_final_target_pub.publish(final_target2);
    cout << "[OptimalTarget]----------------------------------------" << endl;
}
void server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    cout << "[map_input]----------------------------------------" << endl;
    map_data = *msg;
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_resolution = msg -> info.resolution;
    map_origin = msg ->info.origin;
    search_length_cell = search_length/map_resolution;
    half_sq = search_length_cell/2;
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
    cout << "   [robot1path]----------------------------------------" << endl;
    nav_msgs::Path path_tmp = *path_msg;
    float path_length = 0;
    cout << "count: " << robot1path_count << endl;
    for(int j=1;j<path_tmp.poses.size();j++)
    {
        path_length += sqrt(pow(path_tmp.poses[j].pose.position.x-path_tmp.poses[j-1].pose.position.x,2)+pow(path_tmp.poses[j].pose.position.y - path_tmp.poses[j-1].pose.position.y,2));
    }
    cout << "path_length: " << path_length << endl;
    robot1lengths[robot1path_count] = std::make_tuple(robot1path_count, path_length, robot1TARGET[robot1path_count].pose.position.x, robot1TARGET[robot1path_count].pose.position.y);
    robot1path_count++;
    cout << "robot1lengths seq: " << std::get<0>(robot1lengths[robot1path_count]) << endl;
    cout << "robot1lengths size: " << robot1lengths.size() << endl;
    cout << "   [robot1path]----------------------------------------\n" << endl;
}
void server_planning::robot2path(const nav_msgs::Path::ConstPtr &path_msg)
{
    cout << "   [robot2path]----------------------------------------" << endl;
    nav_msgs::Path path_tmp = *path_msg;
    float path_length = 0;
    cout << "count: " << robot2path_count << endl;
    for(int j=1;j<path_tmp.poses.size();j++)
    {
        path_length += sqrt(pow(path_tmp.poses[j].pose.position.x-path_tmp.poses[j-1].pose.position.x,2)+pow(path_tmp.poses[j].pose.position.y - path_tmp.poses[j-1].pose.position.y,2));
    }
    cout << "path_length: " << path_length << endl;
    robot2lengths[robot2path_count] = std::make_tuple(robot2path_count, path_length, robot2TARGET[robot2path_count].pose.position.x, robot2TARGET[robot2path_count].pose.position.y);
    robot2path_count++;
    cout << "robot2lengths seq: " << std::get<0>(robot2lengths[robot2path_count]) << endl; 
    cout << "robot2lengths size: " << robot2lengths.size() << endl;
    cout << "   [robot2path]----------------------------------------\n" << endl;
}
void server_planning::Extraction_Target(void)
{
    cout << "[Extraction_Target]----------------------------------------" << endl;    
    enhance_voronoi_map();
    geometry_msgs::PoseStamped t_target;
    cout << "TARGET size: " << TARGET.size() << endl;
    pre_frox.resize(TARGET.size());
    pre_froy.resize(TARGET.size());
    Extracted_sum = 0;

    for(int i=0; i<fro_num; i++)
    {
        pre_frox[i] = (TARGET[i].pose.position.x-map_origin.position.x)/map_resolution;
        pre_froy[i] = (TARGET[i].pose.position.y-map_origin.position.y)/map_resolution;
    }
    cout << "pre_frox: " << pre_frox.size() << " pre_froy: " << pre_froy.size() << endl;

    int k;
    for(k=0;k<fro_num;k++)
    {
		//std::cout << "pre_frox[k]:" << pre_frox[k] << std::endl;
		if(pre_frox[k]-half_sq < 0)
        {
			//std::cout << "half_leftx:" << half_leftx << std::endl;
			half_leftx = pre_frox[k];
			//std::cout << "half_leftx:" << half_leftx << std::endl;
		}
		else
        {
			half_leftx = half_sq;
		}
		if(pre_frox[k]+half_sq > (map_width-1))
        {
			half_rightx = (map_width-1)-pre_frox[k];
		}
		else
        {
			half_rightx = half_sq;
		}
		if(pre_froy[k]-half_sq < 0)
        {
			half_topy = pre_froy[k];
		}
		else
        {
			half_topy = half_sq;
		}
		if(pre_froy[k]+half_sq > (map_height-1))
        {
			half_bottomy = (map_height-1)-pre_froy[k];
		}
		else
        {
			half_bottomy = half_sq;
		}
		r1_frontier_sum = 0;
        r2_frontier_sum = 0;
		//std::cout << "previent r1_frontier_sum:" << r1_frontier_sum << std::endl;
		for(int i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++)
        {
			for(int j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++)
            {
				r1_frontier_sum+=(-1)*r1_enhanced_Voronoi_grid_array[j][i];
			}
		}
		//std::cout << "after r1_frontier_sum:" << r1_frontier_sum << std::endl;
		if(r1_frontier_sum>128)
        {
			r1_enhanced_Voronoi_grid_array[pre_frox[k]][pre_froy[k]] = 1;
		}
        for(int i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++)
        {
			for(int j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++)
            {
				r2_frontier_sum+=(-1)*r2_enhanced_Voronoi_grid_array[j][i];
			}
		}
		//std::cout << "after r1_frontier_sum:" << r1_frontier_sum << std::endl;
		if(r2_frontier_sum>128)
        {
			r2_enhanced_Voronoi_grid_array[pre_frox[k]][pre_froy[k]] = 1;
		}
	}
    for(int j=0; j < map_height; j++)
    {
        for(int i=0; i < map_width; i++)
        {
            if(r1_enhanced_Voronoi_grid_array[i][j] == 1)
            {
                t_target.pose.position.x = map_resolution * i + map_origin.position.x;
                t_target.pose.position.y = map_resolution * j + map_origin.position.y;
                t_target.pose.orientation.x = 0.0;
                t_target.pose.orientation.y = 0.0;
                t_target.pose.orientation.z = 0.0;
                t_target.pose.orientation.w = 1.0;
                Extraction_Target_r1.push_back(t_target);
            }
        }
    }
    cout << "r1 Extracted target size: " << Extraction_Target_r1.size() << endl;

    
    for(int j=0; j < map_height; j++)
    {
        for(int i=0; i < map_width; i++)
        {
            if(r2_enhanced_Voronoi_grid_array[i][j] == 1)
            {
                t_target.pose.position.x = map_resolution * i + map_origin.position.x;
                t_target.pose.position.y = map_resolution * j + map_origin.position.y;
                t_target.pose.orientation.x = 0.0;
                t_target.pose.orientation.y = 0.0;
                t_target.pose.orientation.z = 0.0;
                t_target.pose.orientation.w = 1.0;
                Extraction_Target_r2.push_back(t_target);
            }
        }
    }
    cout << "r2 Extracted target size: " << Extraction_Target_r2.size() << endl;
    Extracted_sum = Extraction_Target_r1.size() + Extraction_Target_r2.size();
    cout << "Extracted_sum: " << Extracted_sum << endl;
    cout << "[Extraction_Target]----------------------------------------\n" << endl;
}

void server_planning::r1_voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg)
{
    cout << "[r1_voronoi_map_CB]----------------------------------------" << endl;
    r1_map_height = voronoi_map_msg ->info.height;
    r1_map_width = voronoi_map_msg ->info.width;
    cout << "voronoi_map width: " << voronoi_map_msg->info.width << endl;
    cout << "voronoi_map height: " << voronoi_map_msg->info.height << endl;
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
    cout << "voronoi_map width: " << voronoi_map_msg->info.width << endl;
    cout << "voronoi_map height: " << voronoi_map_msg->info.height << endl;
    cout << "voronoi_map resolution: " << voronoi_map_msg->info.resolution << endl;
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
    cout << "[r2_voronoi_map_CB]----------------------------------------\n" << endl;
}

void server_planning::SP_Memory_release(void)
{
    cout << "[Memory_release]----------------------------------------" << endl;
    cout << "r1_voronoi_map_update:" << r1_voronoi_map_update << endl;
    cout << "r1_map_width: " << r1_map_width << endl;
    cout << "map_width: " << map_width << endl;
    if(r1_voronoi_map_update)
    {
        for(int p = 0; p < r1_map_width; p++)
        {
            delete[] r1_Voronoi_grid_array[p];
        }
        cout << "test" << endl;
        delete[] r1_Voronoi_grid_array;
        cout << "test" << endl;
        for(int p = 0; p < map_width; p++)
        {
            delete[] r1_enhanced_Voronoi_grid_array[p];
        }
        cout << "test" << endl;
        delete[] r1_enhanced_Voronoi_grid_array;
        r1_voronoi_map_update = false;
    }
    cout << "r2_vorono_map_update:" << r2_voronoi_map_update << endl;
    cout << "r2_map_width: " << r1_map_width << endl;
    cout << "map_width: " << map_width << endl;
    if(r2_voronoi_map_update)
    {
        for(int p = 0; p < r2_map_width; p++)
        {
            delete[] r2_Voronoi_grid_array[p];
        }
        cout << "test" << endl;
        delete[] r2_Voronoi_grid_array;
        cout << "test" << endl;
        for(int p = 0; p < map_width; p++)
        {
            delete[] r2_enhanced_Voronoi_grid_array[p];
        }
        cout << "test" << endl;
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
    cout << "[FT2robots start]----------------------------------------" << endl;
    robot1lengths.resize(Extracted_sum);
    robot2lengths.resize(Extracted_sum);
    cout << "FT2robots Extracted_sum size: " << Extracted_sum << endl;

    std::string robot1header("/robot1/map");
    std::string robot2header("/robot2/map");
    robot1TARGET.resize(Extracted_sum);
    robot2TARGET.resize(Extracted_sum);
    cout << "test1" << endl;
    if(Extraction_Target_r1.size() != 0)
    {  
        for(int i = 0; i < robot1TARGET.size(); i++)
        {
            robot1TARGET[i].header.frame_id = robot1header;
            robot1TARGET[i].header.seq = i;
            robot1TARGET[i].pose.position.x = Extraction_Target_r1[i].pose.position.x;
            robot1TARGET[i].pose.position.y = Extraction_Target_r1[i].pose.position.y;
            target2robot1.publish(robot1TARGET[i]);
            if(i == robot1TARGET.size()-1) break;
            rate.sleep();
        }
        stop_pose.header.frame_id = robot1header;
        target2robot1.publish(stop_pose);
    }
    cout << "test2" << endl;
    if(Extraction_Target_r2.size() != 0)
    {
        for(int i = 0; i < robot2TARGET.size(); i++)
        {
            robot2TARGET[i].header.frame_id = robot2header;
            robot2TARGET[i].header.seq = i;
            robot2TARGET[i].pose.position.x = Extraction_Target_r2[i].pose.position.x - 3.0;
            robot2TARGET[i].pose.position.y = Extraction_Target_r2[i].pose.position.y;
            target2robot2.publish(robot2TARGET[i]);
            if(i == robot2TARGET.size()-1) break;
            rate.sleep();
        }
        stop_pose.header.frame_id = robot2header;
        target2robot1.publish(stop_pose);
    }
    cout << "[FT2robots end]----------------------------------------\n" << endl;
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
    cout << "   odom_queue_flag: " << odom_queue_flag << endl;
    odom_queue_flag=true;
    cout << "   odom_queue_flag: " << odom_queue_flag << endl;
    cout << "   [create_robot1_grid]----------------------------------------" << endl;
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
    cout << "   odom_queue_flag: " << odom_queue_flag << endl;
    odom_queue_flag=true;
    cout << "   odom_queue_flag: " << odom_queue_flag << endl;
    cout << "   [create_robot2_grid]----------------------------------------" << endl;
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
    cout << "map_height: " << map_height << " map_width: " << map_width << endl;
    cout << "r1_map_height: " << r1_map_height << " r2_map_width: " << r2_map_width << endl;
    //拡張したボロノイ配列にトピックから受け取ったボロノイ図の情報を反映する。
    for(int y = 0; y < r1_map_height; y++)
    {
        for(int x = 0; x < r1_map_width; x++)
        {
            r1_enhanced_Voronoi_grid_array[x][y] = r1_Voronoi_grid_array[x][y];
        }
    }
    for (int y = 0; y < map_height; y++)
    {
        for(int x = 0; x < map_width; x++)
        {
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
    int shift_y;
    shift_y = (int)robot2_init_x*(-1.0);
    cout << "   shift_y: " << shift_y <<endl;
    for(int y = 0; y < r2_map_height; y++)
    {
        for(int x = shift_y; x < r2_map_width; x++)
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
    visualization_msgs::Marker marker;
        marker.header.frame_id = "/server/merge_map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "Extraction_target";
        marker.id = 1;
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

void server_planning::Clear_Vector(void)
{
	pre_frox.clear();
	pre_frox.shrink_to_fit();
	pre_froy.clear();
	pre_froy.shrink_to_fit();
    Extraction_Target_r1.clear();
	Extraction_Target_r1.shrink_to_fit();
    Extraction_Target_r2.clear();
	Extraction_Target_r2.shrink_to_fit();
    robot1lengths.clear();
	robot1lengths.shrink_to_fit();
    robot2lengths.clear();
	robot2lengths.shrink_to_fit();
    robot1TARGET.clear();
    robot1TARGET.shrink_to_fit();
    robot2TARGET.clear();
    robot2TARGET.shrink_to_fit();
}

void server_planning::Clear_Num(void)
{
    robot1path_count = 0;
    robot2path_count = 0;
}
void server_planning::arrive1_flag(const std_msgs::Bool::ConstPtr &msg)
{
    arrive1 = msg -> data;
}
void server_planning::arrive2_flag(const std_msgs::Bool::ConstPtr &msg)
{
    arrive2 = msg -> data;
}


#endif