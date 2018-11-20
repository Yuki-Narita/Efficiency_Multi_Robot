#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <geometry_msgs/PoseArray.h>
#include <geomerry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <iostream>


class Frontier_Search
{

    private:
        ros::NodeHandle ff;
        ros::NodeHandle fp;
        ros::Subscriber subff;
        ros::Publisher pub0;
		geometry_msgs::PoseStamped Pose;
		geometry_msgs::PoseArray poseArray[];

        //vis用
		ros::Publisher vis_pub;		
		ros::NodeHandle vis;
		visualization_msgs::Marker marker;
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
    	float low_left_x;//地図の左下のx座標
    	float low_left_y;//地図の左下のy座標
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
        int **point;
        int **frontier_flag;
		bool input;

		//vis用]
		uint32_t shape = visualization_msgs::Marker::CUBE;

    public:
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
    		subff = ff.subscribe("/map", 1, &Frontier_Search::FSinput, this);
    		pub0 = fp.advertise<geometry_msgs::PoseArray>("/Frontier_Target", 1);
			vis_pub = vis.advertise<visualization_msgs::Marker>("/vis_marker", 1);
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
        void Publish_Data(void);//計算した結果をパブリッシャー用に変換する関数。
		void Publish_marker(void);
		void Memory_release(void);
};

Frontier_Search::~Frontier_Search()
{}
void Frontier_Search::Memory_release(void)
{
		for(int p=0;p<x;p++){
                delete[] map_array[p];
        }
        delete[] map_array;

        for(int p=0;p<x;p++){
                delete[] point[p];
        }
        delete[] point;

        for(int p=0;p<x;p++){
                delete[] frontier_flag[p];
        }
        delete[] frontier_flag;
		std::cout << "メモリ解放" << std::endl;
}

void Frontier_Search::Storage(void)
{

        //int8_t map_array[x][y];//地図を行列に格納
        map_array = new int8_t*[x];
        for(int p=0;p<x;p++){
                map_array[p] = new int8_t[y];
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
					std::cout << "exception:" << map_array[j][i] << std::endl;		
				}
				frontier_flag[j][i] = 0;
				point[j][i] = 0;
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
		std::cout << "pre_frox[k]:" << pre_frox[k] << std::endl;
		if(pre_frox[k]-half_sq < 0){
			std::cout << "half_leftx:" << half_leftx << std::endl;
			half_leftx = pre_frox[k];
			std::cout << "half_leftx:" << half_leftx << std::endl;
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
		else{
			half_bottomy = half_sq;
		}
		
		frontier_sum = 0;
		std::cout << "previent frontier_sum:" << frontier_sum << std::endl;
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				frontier_sum+=point[j][i];
			}
		}
		std::cout << "after frontier_sum:" << frontier_sum << std::endl;

		if(frontier_sum>100){
			point[pre_frox[k]][pre_froy[k]] = 0;
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
	for(int i=0; i<fro_num; i++)
	{
		Pose.header.stamp = ros::Time::now();
		Pose.header.frame_id = "map";
		Pose.pose.position.x = fro_x[i];
		Pose.pose.position.y = fro_y[i];
		Pose.pose.orientation.x = 0.0;
		Pose.pose.orientation.y = 0.0;
		Pose.pose.orientation.z = 0.0;
		Pose.pose.orientation.w = 1.0;
		//std::cout << "fro_x:" << Pose.pose.position.x << "fro_y:" << Pose.pose.position.y << std::endl;
		poseArray.push_back(Pose);
	}
	pub0.publish(poseArray);

}

void Frontier_Search::Publish_marker(void)
{
        uint32_t shape = visualization_msgs::Marker::CUBE_LIST;
        marker.header.frame_id = "map";
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
			std::cout << frontier_flag[j][i];
    		}
			std::cout << std::endl;
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
