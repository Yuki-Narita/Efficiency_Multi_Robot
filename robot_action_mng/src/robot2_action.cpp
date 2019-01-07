#include<robot_action_mng/robot2_action.hpp>

robot2_action::robot2_action()
{
	robot2_pub_nh.setCallbackQueue(&robot2_pub_queue);
	robot2_sub_nh.setCallbackQueue(&robot2_sub_queue);
    robot2_sub = robot2_sub_nh.subscribe("/robot2/final_target", 1, &robot2_action::data_setter, this);
	robot2_pub = robot2_pub_nh.advertise<std_msgs::Int8>("/arrive_flag2", 1);
}
robot2_action::~robot2_action(){}
void robot2_action::data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x = msg -> pose.position.x;
	y = msg -> pose.position.y;
	wait_flag = true;
}

void robot2_action::moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode ){

	std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標を受信＊＊＊＊＊＊＊＊＊＊" << std::endl;
	std::cout << "goalX: " << goalX << " goalY: " << goalY << " mapFrame: " << mapFrame << " movebaseNode: " << movebaseNode << std::endl;

	//define a client for to send goal requests to the move_base server through a SimpleActionClient

	//actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("robot1/move_base", true);
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(movebaseNode, true);

	//wait for the action server to come up//5.0秒まで待つ
	while(!ac.waitForServer(ros::Duration(5.0)) && ros::ok()){
		std::cout << "＊＊＊＊＊＊＊＊＊＊待機中＊＊＊＊＊＊＊＊＊＊" << std::endl;;
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = mapFrame;

	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/

	goal.target_pose.pose.position.x =  goalX;
	goal.target_pose.pose.position.y =  goalY;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = 0.0;
	goal.target_pose.pose.orientation.w = 1.0;

	std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標セット(" << goalX << "," << goalY << ")＊＊＊＊＊＊＊＊＊＊" << std::endl;

	std::cout << "＊＊＊＊＊＊＊＊＊＊経路を作成中＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.sendGoal(goal);
	std::cout << "＊＊＊＊＊＊＊＊＊＊sendend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.waitForResult();
	std::cout << "＊＊＊＊＊＊＊＊＊＊waitend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標に到着＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag2.data = 1;
		robot2_pub.publish(arrive_flag2);
	}
	else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標へのパス生成不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag2.data = 2;
		robot2_pub.publish(arrive_flag2);
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標への移動不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag2.data = 3;
		robot2_pub.publish(arrive_flag2);
		//return res.result;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_action");
	robot2_action R2A;

	R2A.param2.getParam("/multi_planning_server/robot2_action/map_frame2", R2A.frame_id);
	R2A.param2.getParam("/multi_planning_server/robot2_action/move_base_node2", R2A.move_base_node);
	
	while(ros::ok())
    {
		R2A.arrive_flag2.data = false;
        R2A.robot2_sub_queue.callOne();
		if(R2A.wait_flag)
		{
        	R2A.moveToGoal(R2A.x,R2A.y,R2A.frame_id,R2A.move_base_node);
		}
		R2A.wait_flag = false;
		R2A.rate.sleep();
    }
    return 0;
}
