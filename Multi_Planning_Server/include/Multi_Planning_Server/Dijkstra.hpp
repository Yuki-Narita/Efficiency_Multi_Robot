//ダイクストラプログラムの参照元：http://blue-9.hatenadiary.com/entry/2018/01/25/190000
#pragma once

#include<ros/ros.h> 
#include<ros/callback_queue.h>
#include<iostream>
#include<vector>

class Dijkstra
{
    private:
    ros::NodeHandle dn;
    ros::Publisher pub;
    ros::Subscriber sub;
    multi_robot::Target_Info msg;
    std::vector<std::vector<float>> dist;
    std::vector<float> cost;
    std::vector<char> used;
    std::vector<int> via;
    int min;//最小コスト
    int inf = 1000000000;
    int fro_num;//フロンティア領域の個数。
    int x,y;

    bool isinput(void);
    void DKSinit(void);
    void DKSminsearch(void);
    void DKSupdate(void);

    public:
    ros::CallbackQueue queueD;
    
    bool input;
    int Target;

    Dijkstra();
    ~Dijkstra();
    void reset_Flag(void);
    void DKSinput(const multi_robot::Target_Info::ConstPtr& msg);
    void mainloop(void);
};

Dijkstra::Dijkstra()
{   
    dn.setCallbackQueue(&queueD);
 //   pub = dn.advertise<>("Dijkstra", 1000);
    sub = dn.subscribe("Frontier_Search", 1000, &Dijkstra::DKSinput, this);
}

void Dijkstra::mainloop(void)
{
    Dijkstra::DKSinit();
    Dijkstra::DKSminsearch();
    Dijkstra::DKSupdate();
}

void Dijkstra::DKSinit(void)
{
    for(x=0; x < cost.size(); x++)
    {
        cost.push_back(inf);        
    }
        for(x=0; x < used.size(); x++)
    {
        used.push_back(0);        
    }

    for(x=0; x < via.size(); x++)
    {
        via.push_back(-1);        
    }
    for(x=0; x < dist.size(); x++)
    {
        for(y=0; y < dist.front().size(); y++)
        {
            dist.psuh_back(inf);
            dist.front().psuh_back(inf);
        }
    }
}

void Dijkstra::DKSminsearch(void)
{
    while(1)
    {
        /* 未確定の中から距離が最も小さい地点(a)を選んで、その距離を その地点の最小距離として確定します */
        min = inf;
        for(int i = 0; i < N; i++){
          if(!used[i] && min > cost[i]) 
          {
            min = cost[i];
            Target = i;
          }
        }

        /* 全ての地点の最短経路が確定 */
        if(Target == goal)
        {
            return cost[goal];
        }
    
        Dijkstra::DKSupdate();
    }
}

void Dijkstra::DKSupdate(void)
{
    for(int neighboring = 0; neighboring < N; neighboring++)
    {
        if(cost[neighboring] > dist[target][neighboring] + cost[target]) 
        {
            cost[neighboring] = dist[target][neighboring] + cost[target];
            via[neighboring] = target;
        }
    }
    used[target] = 1;
}

void Dijkstra::DKSinput(const multi_robot::Target_Info::ConstPtr& smsg)
{
    msg = smsg;
    input = true;
}

bool Dijkstra::isinput(void)
{
    return input;
}

void Dijkstra::reset_Flag(void)
{
    input = false;
}


