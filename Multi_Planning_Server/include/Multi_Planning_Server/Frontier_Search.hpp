#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

class Frontier_Search
{

    private:
        ros::NodeHandle ff;
        ros::NodeHandle fp;
        ros::Subscriber subff;
        ros::Publisher pub0;		
		geometry_msgs::PoseStamped Pose;
        
        bool input;
        
        //const nav_msgs::MapMetaData msg.info;

        float frontier_x;
        float frontier_y;
		int frontier_num;

        const float m_per_cell;//[m/cell]
    	const float search_len;//障害物を検索する正方形の一辺の長さ[m](都合上10倍して自然数の偶数になる値のみで計算時は+m_per_cell[m]される)
    	const float robot_diameter; //ロボットの直径は0.4[m]
    	const int search_len_cell;//セル換算した正方形の一辺の長さ
    	const int robot_cellsize;//セル換算したロボットサイズ
    	const float low_left_x;//地図の左下のx座標
    	const float low_left_y;//地図の左下のy座標
    	const int half_sq;//正方形の一辺の半分の長さ(セル)


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
    	const int search_width;//フラグの連続を検索するときの線の太さ(奇数)
    	const int search_margin;


        int half_leftx;//四角形の左半分の長さ
        int half_rightx;//四角形の右半分の長さ
        int half_topy;//四角形の上半分の長さ
        int half_bottomy;//四角形の下半分の長さ

	    int8_t **map_array;
        int **point;
        int **frontier_flag;

    public:
        ros::CallbackQueue queueF;
		nav_msgs::OccupancyGrid msg;
		bool stop;
        
        Frontier_Search():
		m_per_cell(msg.info.resolution),
		search_len(0.4),
		robot_diameter(0.4),
		search_len_cell(search_len / m_per_cell),
		robot_cellsize(robot_diameter / m_per_cell),
		low_left_x(msg.info.origin.position.x),
		low_left_y(msg.info.origin.position.y),
		half_sq(search_len_cell / 2),
		pre_fronum(0),
		continuity(0),
    	start_k(0),
    	end_k(0),
    	search_width(3),
    	search_margin(search_width/2),
		x(msg.info.width),
		y(msg.info.height)
		{
			ff.setCallbackQueue(&queueF);
    		subff = ff.subscribe("/map", 1, &Frontier_Search::FSinput, this);
    		pub0 = fp.advertise<geometry_msgs::PoseStamped>("/Frontier_Target", 1000);

		};
        ~Frontier_Search();
        void FSinput(const nav_msgs::OccupancyGrid::ConstPtr& mmsg);//マップの更新を監視
        void Storage(void);//マップ用配列を用意
		void Map_Init(nav_msgs::OccupancyGrid &msg);//配列にマップデータを格納。
		bool isinput(void);//マップが更新され、ＦＳinputが実行されたらフラグを建てる
        void resetFlag(void);//立てたフラグを戻す
        void Boundary_Search(void);//境界判定用関数にするつもりだったが縦横で共通の仕様が考えられなかったのでとりあえず放置
		void Side_Search(void);//横方向に境界を検索する関数
        void Vatical_Search(void);//縦方向に境界を検索する関数
        void Continuity_boundary_Search(void);//境界で連続してフロンティア領域が存在するかを確認する関数。前のBoundary_Searchと同様の理由で放置
		void Side_Continuity_Search(void);//横方向に連続領域を検索
		void Vatical_Continuity_Search(void);//縦方向に連続領域を検索
		void Add_Obstacle(void);//配列に障害物情報を追加する関数
        void Search_Obstacle(void);//
        void Set_Data(void);//計算した結果をパブリッシャー用に変換する関数。
        int Target_Info_Publish(void);//結果をパブリッシュする関数。
};
/*
Frontier_Search::Frontier_Search():
	m_per_cell(msg.info.resolution),
	search_len(0.4),
	robot_diameter(0.4),
	search_len_cell(search_len / m_per_cell),
	robot_cellsize(robot_diameter / m_per_cell),
	low_left_x(msg.info.origin.position.x),
	low_left_y(msg.info.origin.position.y),
	half_sq(search_len_cell / 2),
	pre_fronum(0),
	continuity(0),
    start_k(0),
    end_k(0),
    search_width(3),
    search_margin(search_width/2)
{

}
*/
Frontier_Search::~Frontier_Search()
{
		//newで確保したメモリを開放する
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

	std::cout << "end  :frontier_search" << std::endl;
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

void Frontier_Search::Map_Init(nav_msgs::OccupancyGrid &msg)
{
		std::cout << "start:地図データを配列に格納" << std::endl;

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

void Frontier_Search::Boundary_Search(void)
{
	std::cout << "start:横方向で境界を検索" << std::endl;
	for(int i=0;i<y;i++)
    {
    	for(int j=0;j<(x-1);j++)
        {
      		if(map_array[j][i] == 0 && map_array[j+1][i] == -1)
            {
	      		frontier_flag[j][i] = 1;
		    }
        	else if(map_array[j][i] == -1 && map_array[j+1][i] == 0)
            {
			    frontier_flag[j+1][i] = 1;
		    }
    	}
  	}
	std::cout << "end  :横方向で境界を検索" << " ここまで" << std::endl;
}


void Frontier_Search::FSinput(const nav_msgs::OccupancyGrid::ConstPtr& mmsg)
{
    //ここでサブスクライブされたメッセージのデータをこのクラスのメンバ変数などに代入して使えるようにする。
	msg = *mmsg;
	input = true;
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

void Frontier_Search::Continuity_boundary_Search(void)
{
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
}


void Frontier_Search::Search_Obstacle(void)
{
	std::cout << "start:未探査領域周辺の障害物を検索" << std::endl;
    for(k=0;k<pre_fronum;k++){
		
		if(pre_frox[k]-half_sq < 0){
			half_leftx = pre_frox[k];
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
		
		for(i=(pre_froy[k]-half_topy);i<(pre_froy[k]+half_bottomy+1);i++){
			for(j=(pre_frox[k]-half_leftx);j<(pre_frox[k]+half_rightx+1);j++){
				frontier_sum+=point[j][i];
			}
		}

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

void Frontier_Search::Set_Data(void)
{
	Pose.header.seq++;
	Pose.header.stamp = ros::Time::now();
	Pose.header.frame_id = "goal";
	Pose.pose.position.x = fro_x.at(1);
	Pose.pose.position.y = fro_y.at(1);
}

int Frontier_Search::Target_Info_Publish(void)
{
	pub0.publish(Pose);
}

void Frontier_Search::Side_Search(void)
{

	std::cout << "start:横方向で境界を検索" << std::endl;

	for(i=0;i<y;i++){
    		for(j=0;j<(x-1);j++){
      			if(map_array[j][i] == 0 && map_array[j+1][i] == -1){
	       			frontier_flag[j][i] = 1;
			}
        		else if(map_array[j][i] == -1 && map_array[j+1][i] == 0){
				frontier_flag[j+1][i] = 1;	
			}
    		}
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
    		}
  	}

	std::cout << "end  :縦方向で境界を検索" << std::endl;
}

void Frontier_Search::Side_Continuity_Search(void)
{

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

void Frontier_Search::Vatical_Continuity_Search(void)
{

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
	
	std::cout << "end  :縦方向で境界が連続している場所を検索" << std::endl;

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
