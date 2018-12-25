#include<robot_action_mng/robot1_action.hpp>

robot1_action::robot1_action()
{
	ros::NodeHandle param1("~/robot1/");
    ros::NodeHandle robot1("~/robot1/");
	robot1.setCallbackQueue(&robot1_queue);
    robot1_sub = robot1.subscribe("/robot1/final_target", 1, &robot1_action::data_setter, this);
	robot1_pub = robot1.advertise<std_msgs::Bool>("arrive_flag", 1);
    param1.getParam("frame_id1", frame_id);
	param1.getParam("move_base_node1", move_base_node);
	
}
robot1_action::~robot1_action(){}
void robot1_action::data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x = msg -> pose.position.x;
	y = msg -> pose.position.y;
	wait_flag = true;

}

void robot1_action::moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode ){

	std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標を受信＊＊＊＊＊＊＊＊＊＊" << std::endl;

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
		arrive_flag1.data = true;
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標への移動不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag1.data = false;
		//return res.result;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_action");
	std::cout << "test" << std::endl;
	robot1_action R1A;
	std::cout << "test" << std::endl;
    while(ros::ok())
    {
		std::cout << "test" << std::endl;
		R1A.arrive_flag1.data = false;
		std::cout << "test" << std::endl;
        R1A.robot1_queue.callOne();
		std::cout << "test" << std::endl;
		if(R1A.wait_flag)
		{
        	R1A.moveToGoal(R1A.x,R1A.y,R1A.frame_id,R1A.move_base_node);
		}
		R1A.wait_flag = false;
		std::cout << "end loop" << std::endl;
    }
    return 0;
}
