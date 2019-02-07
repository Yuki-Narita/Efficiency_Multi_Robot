#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

using std::cout;
using std::endl;

class Frontier_Search
{

    private:
        ros::NodeHandle ff;
        ros::NodeHandle fp;
        ros::Subscriber subff;
        ros::Publisher pub0;
		geometry_msgs::Pose Pose;	

        //vis用
		ros::Publisher vis_pub;		
		ros::NodeHandle vis;
		//visualization_msgs::Marker marker;
        //

        //const nav_msgs::MapMetaData msg.info;



        float frontier_x;
        float frontier_y;
		int frontier_num;

        float m_per_cell;//[m/cell]
    	float search_len;//障害物を検索する正方形の一辺の長さ[m](都合上10倍して自然数の偶数になる値のみで計算時は+m_per_cell[m]される)
    	float robot_diameter; //ロボットの直径は0.4[m]
    	int search_len_cell;//セル換算した正方形の一辺の長さ
    	int robot_cellsize;//セル換算したロボットサイズ
    	float low_left_x;//マップ中心から地図の左下までのx座標
    	float low_left_y;//マップ中心から地図の左下までのy座標
    	int half_sq;//正方形の一辺の半分の長さ(セル)


    	int frontier_sum;//フラグが続いているかの判定用
    	float frontier_center;//フロンティア境界の中点
    	int flo2int;
    	std::vector<int> pre_frox;//未探査領域のx座標を一時保存
    	std::vector<int> pre_froy;//未探査領域のy座標を一時保存
    	int pre_fronum;//未探査領域の個数を一時保存
    	
		int fro_num;//見つけたフロンティア領域の個数
		std::vector<float> fro_x;//見つけたフロンティア領域のｘ座標
		std::vector<float> fro_y;//見つけたフロンティア領域のｙ座標

    	int x,y,i,j,k,v;
    	int continuity;
    	int start_k;
    	int end_k;
    	int search_width;//フラグの連続を検索するときの線の太さ(奇数)
    	int search_margin;


        int half_leftx;//四角形の左半分の長さ
        int half_rightx;//四角形の右半分の長さ
        int half_topy;//四角形の上半分の長さ
        int half_bottomy;//四角形の下半分の長さ

	    int8_t **map_array;
		int **robot1_costmap_array;
		int **robot2_costmap_array;
        int **point;
        int **frontier_flag;
		int **r1_enhanced_costmap_array;
		int **r2_enhanced_costmap_array;
		bool input;

		//vis用]
		uint32_t shape = visualization_msgs::Marker::CUBE;

    public:
		//コストマップ購読用
		ros::NodeHandle robot1_costmap_nh;
		ros::Subscriber robot1_costmao_sub;
		ros::CallbackQueue robot1_costmap_queue;
		ros::NodeHandle robot2_costmap_nh;
		ros::Subscriber robot2_costmao_sub;
		ros::CallbackQueue robot2_costmap_queue;
		int robot1_costmap_height;
		int robot1_costmap_width;
		int robot2_costmap_height;
		int robot2_costmap_width;
		std::vector<int> robot1_costmap_data;
		std::vector<int> robot2_costmap_data;

		//拡張コストマップのグローバルコストマップへのシフトのために必要なロボットの初期位置用の変数
		ros::NodeHandle get_param_nh;
		int robot1_init_x;
		int robot1_init_y;
		int robot2_init_x;
		int robot2_init_y;

        ros::CallbackQueue queueF;
		nav_msgs::OccupancyGrid msg;
		bool stop;
        
        Frontier_Search():
		search_len(0.4),
		robot_diameter(0.4),
		pre_fronum(0),
		continuity(0),
    	start_k(0),
    	end_k(0),
    	search_width(3)
		{
			ff.setCallbackQueue(&queueF);
			robot1_costmap_nh.setCallbackQueue(&robot1_costmap_queue);
			robot2_costmap_nh.setCallbackQueue(&robot2_costmap_queue);
    		subff = ff.subscribe("/server/grid_map_merge/merge_map", 1, &Frontier_Search::FSinput, this); //購読先がグローバルマップ
			//subff = ff.subscribe("/robot2/move_base/local_costmap/costmap", 1, &Frontier_Search::FSinput, this); //購読先がコストマップ
			robot1_costmao_sub = robot1_costmap_nh.subscribe("/robot1/move_base/global_costmap/costmap", 1, &Frontier_Search::Robot1_Costmap_Data_Setter, this);
			robot1_costmao_sub = robot2_costmap_nh.subscribe("/robot2/move_base/global_costmap/costmap", 1, &Frontier_Search::Robot2_Costmap_Data_Setter, this);
    		pub0 = fp.advertise<geometry_msgs::PoseArray>("/Frontier_Target", 1);
			vis_pub = vis.advertise<visualization_msgs::Marker>("/vis_marker/Frontier", 1);
			get_param_nh.getParam("/robot1_init_x",robot1_init_x);
			get_param_nh.getParam("/robot1_init_y",robot1_init_y);
			get_param_nh.getParam("/robot2_init_x",robot2_init_x);
			get_param_nh.getParam("/robot2_init_y",robot2_init_y);
			std::cout << "search_len :" << search_len << std::endl;
			std::cout << "robot_diameter:" << robot_diameter << std::endl;
			std::cout << "pre_fronum:" << pre_fronum << std::endl;
			std::cout << "continuity:" << continuity << std::endl;
			std::cout << "start_k:" << start_k << std::endl;
			std::cout << "end_k:" << end_k << std::endl;
			std::cout << "search_width:" << search_width << std::endl;
		};
        ~Frontier_Search();
        void FSinput(const nav_msgs::OccupancyGrid::ConstPtr& mmsg);//マップの更新を監視
        void Storage(void);//マップ用配列を用意
		void Map_Init(nav_msgs::OccupancyGrid &msg);//配列にマップデータを格納。
		bool isinput(void);//マップが更新され、ＦＳinputが実行されたらフラグを建てる
        void resetFlag(void);//立てたフラグを戻す
        void Side_Search(void);//横方向に境界を検索する関数
        void Vatical_Search(void);//縦方向に境界を検索する関数
        void Side_Continuity_Search(void);//横方向に連続領域を検索
		void Vatical_Continuity_Search(void);//縦方向に連続領域を検索
		void Add_Obstacle(void);//配列に障害物情報を追加する関数
        void Search_Obstacle(void);//
		void Robot1_Costmap_Data_Setter(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void Robot2_Costmap_Data_Setter(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void r1_enhanced_costmap(const std::vector<int> &map_data);
		void r2_enhanced_costmap(const std::vector<int> &map_data);
        void Publish_Data(void);//計算した結果をパブリッシャー用に変換する関数。
		void Publish_marker(void);
		void Memory_release(void);
		void Clear_Vector(void);
};

Frontier_Search::~Frontier_Search()
{}

void Frontier_Search::Clear_Vector(void)
{
	pre_frox.clear();
	pre_frox.shrink_to_fit();

	pre_froy.clear();
	pre_froy.shrink_to_fit();

	fro_x.clear();
	fro_x.shrink_to_fit();

	fro_y.clear();
	fro_y.shrink_to_fit();
}

void Frontier_Search::Memory_release(void)
{
		for(int p=0;p<x;p++){
                delete[] map_array[p];
        }
        delete[] map_array;

		for(int p=0;p<robot1_costmap_width;p++){
                delete[] robot1_costmap_array[p];
        }
        delete[] robot1_costmap_array;
		
		for(int p=0;p<robot2_costmap_width;p++){
                delete[] robot2_costmap_array[p];
        }
        delete[] robot2_costmap_array;
        
		for(int p=0;p<x;p++){
                delete[] point[p];
        }
        delete[] point;

        for(int p=0;p<x;p++){
                delete[] frontier_flag[p];
        }
        delete[] frontier_flag;
		for(int p=0;p<x;p++)
		{
			delete[] r1_enhanced_costmap_array[p];
		}
		for(int p=0;p<x;p++)
		{
			delete[] r2_enhanced_costmap_array[p];
		}
		std::cout << "メモリ解放" << std::endl;
}

void Frontier_Search::Storage(void)
{

        //int8_t map_array[x][y];//地図を行列に格納
        map_array = new int8_t*[x];
        for(int p=0;p<x;p++){
                map_array[p] = new int8_t[y];
        }
		//ロボット１用のコストマップ
		robot1_costmap_array = new int*[robot1_costmap_width];
        for(int p=0;p<robot1_costmap_width;p++){
                robot1_costmap_array[p] = new int[robot1_costmap_height];
        }
		//ロボット２用のコストマップ
		robot2_costmap_array = new int*[robot2_costmap_width];
        for(int p=0;p<robot2_costmap_width;p++){
                robot2_costmap_array[p] = new int[robot2_costmap_height];
        }
		//ロボット１用の拡張コストマップ
		r1_enhanced_costmap_array = new int*[x];
        for(int p=0;p<x;p++){
                r1_enhanced_costmap_array[p] = new int[y];
        }
		//ロボット２用の拡張コストマップ
		r2_enhanced_costmap_array = new int*[x];
        for(int p=0;p<x;p++){
                r2_enhanced_costmap_array[p] = new int[y];
        }

        //int frontier_flag[x][y];//探査済みと未探査の境界を判定するフラグ
        frontier_flag = new int*[x];
        for(int p=0;p<x;p++){
                frontier_flag[p] = new int[y];
        }

        //int point[x][y];//未探査領域の近くに障害物があるか判定する用
        point = new int*[x];
        for(int p=0;p<x;p++){
                point[p] = new int[y];
        }
	//void Map_Init(msg);
}

void Frontier_Search::Map_Init(nav_msgs::OccupancyGrid& msg)
{
	k=0;
	std::cout << "start:地図データを配列に格納" << std::endl;
	std::cout << "X:" << x << "Y:" << y << "K:" << k << std::endl; 
	for(i=0;i<y;i++){
    	for(j=0;j<x;j++){
      		map_array[j][i] = msg.data[k];
			if(map_array[j][i]!=0 && map_array[j][i]!=100 && map_array[j][i]!=-1)
			{	
				//std::cout << "exception:" << map_array[j][i] << std::endl;		
			}
			frontier_flag[j][i] = 0;
			point[j][i] = 0;
			r1_enhanced_costmap_array[j][i] = 0;
			r2_enhanced_costmap_array[j][i] = 0;
      		k++;
    	}
  	}
	
	std::cout << "end  :地図データを配列に格納" << std::endl;
}

void Frontier_Search::FSinput(const nav_msgs::OccupancyGrid::ConstPtr& mmsg)
{
    //ここでサブスクライブされたメッセージのデータをこのクラスのメンバ変数などに代入して使えるようにする。
	msg = *mmsg;
	x = msg.info.width;
	y = msg.info.height;
	m_per_cell = msg.info.resolution;
	low_left_x = msg.info.origin.position.x;
	low_left_y = msg.info.origin.position.y;
	search_len_cell = search_len / m_per_cell;
	robot_cellsize = robot_diameter / m_per_cell;
	half_sq = search_len_cell / 2;
	search_margin = search_width /2;
	input = true;

	std::cout << "map hight:" << msg.info.height << "y:" << y << std::endl;
	std::cout << "map width:" << msg.info.width << "x:" << x << std::endl;
	std::cout << "m_per_cell:" << m_per_cell << std::endl;
	std::cout << "search_len_cell:" << search_len_cell << std::endl;
	std::cout << "robot_cellsize:" << robot_cellsize << std::endl;
	std::cout << "half_sq:" << half_sq << std::endl;
	std::cout << "search_margin:" << search_margin << std::endl;
	std::cout << "FSinput Done." << std::endl;
}

bool Frontier_Search::isinput(void)
{
	return input;
}

void Frontier_Search::resetFlag(void)
{
    input = false;
}

void Frontier_Search::Search_Obstacle(void)
{
	std::cout << "start:未探査領域周辺の障害物を検索" << std::endl;
	std::cout << "pre_fronum:" << pre_fronum << std::endl;
    for(k=0;k<pre_fronum;k++){
		//std::cout << "pre_frox[k]:" << pre_frox[k] << std::endl;
		if(pre_frox[k]-half_sq < 0){
			//std::cout << "half_leftx:" << half_leftx << std::endl;
			half_leftx = pre_frox[k];
			//std::cout << "half_leftx:" << half_leftx << std::endl;
		}
		else{
			half_leftx = half_sq;
		}

		if(pre_frox[k]+half_sq > (x-1)){
			half_rightx = (x-1)-pre_frox[k];
		}
		else{
			half_rightx = half_sq;
		}
		if(pre_froy[k]-half_sq < 0){
			half_topy = pre_froy[k];
		}
		else{
			half_topy = half_sq;
		}
		if(pre_froy[k]+half_sq > (y-1)){
			half_bottomy = (y-1)-pre_froy[k];
		}
		else
		{
			half_bottomy = half_sq;
		}
		
		frontier_sum = 0;
		//std::cout << "previent frontier_sum:" << frontier_sum << std::endl;
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				frontier_sum+=point[j][i];
			}
		}
		//std::cout << "after frontier_sum:" << frontier_sum << std::endl;
		if(frontier_sum>100){
			point[pre_frox[k]][pre_froy[k]] = 0;
		}
		//コストマップと比較して目的地を絞る処理をする。
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				if(r1_enhanced_costmap_array[j][i] != 0)
				{
					point[pre_frox[k]][pre_froy[k]] = 0;
				}
				if(r2_enhanced_costmap_array[j][i] != 0)
				{
					point[pre_frox[k]][pre_froy[k]] = 0;
				}
			}
		}

	}
		//最終的な未探査領域を配列に格納
	for(j=0;j<x;j++){
    	for(i=0;i<y;i++){
      		if(point[j][i] == 1){
	    		fro_x.push_back(m_per_cell * j + low_left_x);
				fro_y.push_back(m_per_cell * i + low_left_y);
				fro_num++;
			}
    	}
  	}

	std::cout << "障害物判定前の未探査領域の個数 " << pre_fronum << std::endl;
	std::cout << "障害物判定後の未探査領域の個数 " << fro_num << std::endl;

	std::cout << "end  :未探査領域周辺の障害物を検索" << std::endl;
	
	if(fro_num == 0){
		stop = true;
		return;
	}
	
}

void Frontier_Search::Publish_Data(void)
{
	geometry_msgs::PoseArray poseArray;
	for(int i=0; i<fro_num; i++)
	{
		poseArray.header.stamp = ros::Time::now();
		poseArray.header.frame_id = "/server/merge_map";
		Pose.position.x = fro_x[i];
		Pose.position.y = fro_y[i];
		Pose.orientation.x = 0.0;
		Pose.orientation.y = 0.0;
		Pose.orientation.z = 0.0;
		Pose.orientation.w = 1.0;
		//std::cout << "fro_x:" << Pose.pose.position.x << "fro_y:" << Pose.pose.position.y << std::endl;
		poseArray.poses.push_back(Pose);
	}
	pub0.publish(poseArray);
	cout << "pose size: " << poseArray.poses.size() << endl;
}

void Frontier_Search::Publish_marker(void)
{
	visualization_msgs::Marker marker;
	uint32_t shape = visualization_msgs::Marker::CUBE_LIST;
	marker.header.frame_id = "/server/merge_map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "Frontier";
	marker.id = 0;
	marker.type = shape;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.lifetime = ros::Duration();

	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;

	geometry_msgs::Point p;
	for(int i=0; i < fro_num;i++)
	{
		p.x = fro_x[i];
		p.y = fro_y[i];
		marker.points.push_back(p);
	}
	vis_pub.publish(marker);

}

void Frontier_Search::Side_Search(void)
{

	std::cout << "start:横方向で境界を検索" << std::endl;
	std::cout << "Side_Search\t" << "X:" << x << "Y:" << y << std::endl;
	for(i=0;i<y;i++)
	{
    		for(j=0;j<(x-1);j++)
			{
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1)
				{
					frontier_flag[j][i] = 1;
				}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0)
				{
					frontier_flag[j+1][i] = 1;
				}
					//std::cout << frontier_flag[j][i] << " ";
    		}
			//std::cout << std::endl;
  	}
	std::cout << "end  :横方向で境界を検索" << " ここまで" << std::endl;

}

void Frontier_Search::Vatical_Search(void)
{
		std::cout << "start:縦方向で境界を検索" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<(y-1);i++){
      			if(map_array[j][i] == 0 && map_array[j][i+1] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j][i+1] == 0){
				frontier_flag[j][i+1] = 1;	
			}
			//std::cout << frontier_flag[j][i];
    		}
			//std::cout << std::endl;
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;
}

void Frontier_Search::Vatical_Continuity_Search(void)
{
	std::cout << "start:  縦方向で境界が連続している場所を検索" << std::endl;

	for(j=search_margin;j<(x-search_margin);j=j+search_width){
		k = 0;
		
		while(k < y && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[j+v][k];
			}
			if(frontier_sum > 0){
				start_k = k;
				continuity = 0;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < y){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[j+v][k];
						}
					}
					else{	
						k++;
						break;
					}
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					//std::cout << "Vatical_Continuity << " << continuity << ", start_k << " << start_k << ", end_k << " << end_k << std::endl;
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[j][flo2int] = 1;
					pre_frox.push_back(j);
					pre_froy.push_back(flo2int);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

}

void Frontier_Search::Side_Continuity_Search(void)
{
	std::cout << "start  :横方向で境界が連続している場所を検索" << std::endl;
	pre_fronum = 0;
	fro_num = 0;
	continuity = 0;
	start_k = 0;
	end_k = 0;
	std::cout << "search_margin: " << search_margin << std::endl;
	std::cout << "search_margin: " << search_margin << std::endl;
	for(i=search_margin;i<(y-search_margin);i=i+search_width){
		k = 0;
		while(k < x && ros::ok()){
			for(v=-search_margin;v<=search_margin;v++){
				frontier_sum += frontier_flag[k][i+v];
			}
			if(frontier_sum > 0){
				start_k = k;
				continuity = 0;
				while(frontier_sum > 0 && ros::ok()){
					frontier_sum = 0;
					continuity++;
					if(k < x){
						k++;
						for(v=-search_margin;v<=search_margin;v++){
							frontier_sum += frontier_flag[k][i+v];
						}
					}
					else{	
						k++;
						break;
					}
					//std::cout << k << "," << x << std::endl;
				}
				end_k = k-1;
				if(continuity >= robot_cellsize){
					//std::cout << "Side_Continuity << " << continuity << ", start_k << " << start_k << ", end_k << " << end_k << std::endl;
					frontier_center = (start_k + end_k)/2;
					flo2int = frontier_center;
					point[flo2int][i] = 1;
					pre_frox.push_back(flo2int);
					pre_froy.push_back(i);
					pre_fronum++;
				}
			}
			else{			
				k++;
			}
		}
	}
	
	std::cout << "end  :横方向で境界が連続している場所を検索" << std::endl;

}

void Frontier_Search::Add_Obstacle(void)
{
	std::cout << "start:障害物情報を追加" << std::endl;

	for(j=0;j<x;j++){
    		for(i=0;i<y;i++){
      			if(map_array[j][i] == 100){
	       			point[j][i] = 100;
				}
    		}
  	}
	std::cout << "end  :障害物情報を追加" << std::endl;
}
void Frontier_Search::Robot1_Costmap_Data_Setter(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	robot1_costmap_width = msg -> info.width;
	robot1_costmap_height = msg -> info.height;
	for(i=0; i<robot1_costmap_height*robot1_costmap_width; i++)
	{
		robot1_costmap_data[i] = msg->data[i];
	}


}
void Frontier_Search::Robot2_Costmap_Data_Setter(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	robot2_costmap_width = msg -> info.width;
	robot2_costmap_height = msg -> info.height;
	for(i=0; i<robot2_costmap_height*robot2_costmap_width; i++)
	{
		robot2_costmap_data[i] = msg->data[i];
	}
}

void Frontier_Search::r1_enhanced_costmap(const std::vector<int> &map_data)
{
	int k=0;
	std::cout << "start:ロボット１のコストマップデータを配列に格納" << std::endl;
	std::cout << "X:" << x << "Y:" << y << "K:" << k << std::endl; 
	for(i=0;i<robot1_costmap_height;i++){
    	for(j=0;j<robot1_costmap_width;j++){
      		robot1_costmap_array[j][i] = map_data[k];
			k++;
    	}
  	}
	for(i=0;i<y;i++){
    	for(j=0;j<x;j++){
      		r1_enhanced_costmap_array[j][i] = robot1_costmap_array[j][i];
    	}
  	}
	std::cout << "end  :ロボット２のコストマップデータを配列に格納" << std::endl;
}
void Frontier_Search::r2_enhanced_costmap(const std::vector<int> &map_data)
{
	int k=0;
	std::cout << "start:ロボット１のコストマップデータを配列に格納" << std::endl;
	std::cout << "X:" << x << "Y:" << y << "K:" << k << std::endl; 
	for(i=0;i<robot2_costmap_height;i++){
    	for(j=0;j<robot2_costmap_width;j++){
      		robot2_costmap_array[j][i] = map_data[k];
			k++;
    	}
  	}
	int shift_x;
	shift_x = (int)robot2_init_x*(-1.0);
	for(i=0;i<y;i++){
    	for(j=shift_x;j<x;j++){
      		r1_enhanced_costmap_array[j][i] = robot1_costmap_array[j][i];
    	}
  	}
	std::cout << "end  :ロボット２のコストマップデータを配列に格納" << std::endl;
}