#ifndef _SERVER_PLANNING_HPP_
#define _SERVER_PLANNING_HPP_

#include<ros/ros.h>
#include<ros/callback_queue.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Odometry.h>
#include<std_srvs/Empty.h>
#include<std_msgs/String.h>
#include<sstream>
#include<string>

bool arrive_flag;//ロボットが目的地に到着したかを判定する用
int robot_num;//ロボットの個数、台数。
int fro_num;//フロンティア領域の個数。
int given_robot_num;//launchの引数として与えられたロボットの台数。
nav_msgs::OccupancyGrid map_data;//大元のmapトピックのマップ情報
uint32_t map_width;
uint32_t map_height;
int **Voronoi_grid_array;
uint8_t **Frontier_array;
float map_resolution;

class server_planning
{
    private:
    std::vector<geometry_msgs::PoseStamped> TARGET;
    nav_msgs::Odometry robot_odom;

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

    ros::Subscriber path_sub;
    ros::Subscriber Target_sub;
    ros::Subscriber map_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber voronoi_grid_sub;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle fn;
    ros::NodeHandle mn;
    ros::NodeHandle on;
    ros::NodeHandle voronoi_map_nh;
    ros::CallbackQueue queueS;
    ros::CallbackQueue queueF;
    ros::CallbackQueue queueM;
    ros::CallbackQueue queueO;
    ros::CallbackQueue voronoi_map_queue;
    std_msgs::String pub_msg;
    std_msgs::String sub_msg;
    
    bool isinput;
    bool turn_fin;
    std::vector<geometry_msgs::PoseStamped> Extraction_Target_m;
};

server_planning::server_planning()
{
    nh.setCallbackQueue(&queueS);
    fn.setCallbackQueue(&queueF);
    mn.setCallbackQueue(&queueM);
    on.setCallbackQueue(&queueO);
    voronoi_map_nh.setCallbackQueue(&voronoi_map_queue);

    path_sub=nh.subscribe("/move_base/VoronoiPlanner/path", 100, &server_planning::OptimalTarget, this);
    Target_sub=fn.subscribe("/Frontier_Target", 100, &server_planning::frontier_target_CB, this);
    map_sub=mn.subscribe("/map", 100, &server_planning::map_input, this);
    odom_sub=on.subscribe("/odom", 100, &server_planning::odomCB, this);
    voronoi_grid_sub=voronoi_map_nh.subscribe("/move_base/VoronoiPLanner/grid", 100, &server_planning::voronoi_map_CB, this);
}
server_planning::~server_planning()
{}
void server_planning::odomCB(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    robot_odom = *odom_msg;
}

void server_planning::frontier_target_CB(const geometry_msgs::PoseArray &Target)
{    
    fro_num = Target.poses.size();
    TARGET.resize(Target.poses.size());
    for(int i = 0; i < Target.poses.size(); i++)
    {
        TARGET[i].header = Target.header;
        TARGET[i].pose = Target.poses[i];
    }

    /*
    int i = 0;
    for(std::vector<int>::const_iterator it = Target.begin(); it != Target.end(); ++it)
    {
        TARGET[i] = *it;
        std::cout << "frontier_target_CB TARGET:" << TARGET[i] << std::endl;
        i++;
    }
    std::cout << "server_plannning getfrontier is done." << std::endl;
    frontier_target2map(TARGET);
    */
}
void frontier_target2map(const std::vector<geometry_msgs::PoseStamped>& Target)
{
    //Frontier_map用の配列を確保
    Frontier_array = new uint8_t*[map_width];
    for(uint8_t p = 0; p < map_height; p++)
    {
        Frontier_array[p] = new uint8_t[map_height];
    }
    //Targetの型をfloatからintにする（map配列に座標を変換してその中の値を参照するため）
    std::vector<int> frontier_x;
    std::vector<int> frontier_y;

    frontier_x.resize(Target.size());
    frontier_y.resize(Target.size());

    for(int i = 0; i < Target.size(); i++)
    {
        frontier_x[i] = (int)Target[i].pose.position.x+1;
        frontier_y[i] = (int)Target[i].pose.position.y+1;
    }
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
}
void server_planning::OptimalTarget(const geometry_msgs::PoseStamped::ConstPtr &Target)
{
    //ロボットの自己位置と各目標地点とのパスを取得してロボットの自己位置に対する最も近い
}
void server_planning::map_input(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map_data = *msg;
    map_width = msg->info.width;
    map_height = msg->info.height;
    map_resolution = msg -> info.resolution;
    isinput = true;
    ROS_INFO_STREAM("Map is updated");
}
bool server_planning::map_isinput(void)
{
    return isinput;
}
void server_planning::turn_fin_CB(const std_msgs::String::ConstPtr &msg)
{
    sub_msg = *msg;
    turn_fin = true;
    std::cout << "turn_fin_CB was done." << std::endl;
}
/*
void server_planning::robot_sort1(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier)
{
    std::vector<string> sort1_robot_name = *robot_name;
    geometry_msgs::PoseArray sort1_Frontier = *Frontier;

}
void server_planning::robot_sort2(const std::vector<string>& robot_name, const geometry_msgs::PoseArray& Frontier)
{

}
*/
void server_planning::Extraction_Target(void)
{
    geometry_msgs::PoseStamped t_target;
    for(uint32_t i=0; i < map_width; i++)
    {
        for(uint32_t j=0; j < map_height; j++)
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
}

void server_planning::voronoi_map_CB(const nav_msgs::OccupancyGrid::ConstPtr& voronoi_map_msg)
{
    //ボロノイグリッド格納用の配列を確保
    Voronoi_grid_array = new int*[voronoi_map_msg->info.width];
    for(uint8_t p = 0; p < voronoi_map_msg->info.width; p++)
    {
        Voronoi_grid_array[p] = new int [voronoi_map_msg->info.height];
    }
    //ボロノイグリッドを配列に格納
    for(uint8_t i = 0; i < voronoi_map_msg->info.width; i++)
    {
        for(uint8_t j = 0; j < voronoi_map_msg->info.height; j++)
        {
            Voronoi_grid_array[i][j]=voronoi_map_msg->data[voronoi_map_msg->info.width*j+i];
        }
    }
}

void server_planning::SP_Memory_release(void)
{
    for(uint32_t p = 0; p < map_width; p++)
    {
        delete[] Voronoi_grid_array[p];
    }
    delete[] Voronoi_grid_array;
    for(uint32_t p = 0; p < map_width; p++)
    {
        delete[] Frontier_array[p];
    }
    delete[] Frontier_array;
}
#endif