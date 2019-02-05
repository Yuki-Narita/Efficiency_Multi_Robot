#include<robot_action_mng/robot1_action.hpp>

robot1_action::robot1_action()
{

	robot1_pub_nh.setCallbackQueue(&robot1_pub_queue);
	robot1_sub_nh.setCallbackQueue(&robot1_sub_queue);
	robot1_odom_sub_nh.setCallbackQueue(&robot1_odom_queue);
    robot1_sub = robot1_sub_nh.subscribe("/robot1/final_target", 1, &robot1_action::data_setter, this);
	//robot1_odom_sub = robot1_odom_sub_nh.subscribe("/robot1/odom", 1, &robot1_action::escape_robot_stack, this);
	robot1_pub = robot1_pub_nh.advertise<std_msgs::Int8>("/arrive_flag1", 1);
	robot1_test_pub = robot1_test_pub_nh.advertise<geometry_msgs::PoseStamped>("/robot1/move_base_simple/goal", 1);
	
	robot1GoalPub = robot1GoalNh.advertise<visualization_msgs::Marker>("/robot1/action_goal",1);
}
robot1_action::~robot1_action(){}
/*
void robot1_action::escape_robot_stack(const nav_msgs::Odometry::ConstPtr &odom)
{
	std::cout << "escape_robot_stack start" << std::endl;
	static nav_msgs::Odometry previous, current;
	previous = current;
	current = *odom;

	float diff_x, diff_y;
	int check_diff_x, check_diff_y;
	diff_x = current.pose.pose.position.x - previous.pose.pose.position.x;
	diff_y = current.pose.pose.position.y - previous.pose.pose.position.y;
	check_diff_x = diff_x * 1000;
	check_diff_y = diff_y * 1000;

	if(check_diff_x == 0 && check_diff_y == 0)
	{
		std::cout << "if" << std::endl;
		static int count = 0;
		count++;
		if(count == 10)
		{
			goalstate = actionlib::SimpleClientGoalState::ABORTED;
			check_robot_stack = true;
			std::cout << "escape stack" << std::endl;
		}
	}	
	std::cout << "escape_robot_stack end" << std::endl;
}
*/
void robot1_action::setGoalMarker(const double x,const double y, const std::string frameId)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frameId;
	marker.header.stamp = ros::Time::now();

	marker.ns = "robot1_goal";
	marker.id = 0;
	marker.lifetime = ros::Duration(0);
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::CUBE;

	marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

	marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

	marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

	robot1GoalPub.publish(marker);
}
void robot1_action::data_setter(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
	x = msg -> pose.position.x;
	y = msg -> pose.position.y;
	wait_flag = true;
}
void robot1_action::moveToGoal(double goalX,double goalY,std::string mapFrame,std::string movebaseNode ){

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

	setGoalMarker(goalX,goalY,mapFrame);
	geometry_msgs::PoseStamped goalpose;
	goalpose.pose.position.x = goalX;
	goalpose.pose.position.y = goalY;
	goalpose.pose.position.z = 0.0;
	goalpose.pose.orientation.x=0.0;
	goalpose.pose.orientation.y=0.0;
	goalpose.pose.orientation.z=0.0;
	goalpose.pose.orientation.w=1.0;
	goalpose.header.frame_id = mapFrame;

	std::cout << "＊＊＊＊＊＊＊＊＊＊経路を作成中＊＊＊＊＊＊＊＊＊＊" << std::endl;
	ac.sendGoal(goal);
	std::cout << "＊＊＊＊＊＊＊＊＊＊sendend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	while(ac.waitForResult(ros::Duration(1.0)) != true && check_robot_stack != true && ros::ok())
	{
		actionlib::SimpleClientGoalState check_state = ac.getState();
		std::cout << "state: " <<  check_state.toString() << std::endl;
		robot1_odom_queue.callOne();
	}
	check_robot_stack = false;
	std::cout << "＊＊＊＊＊＊＊＊＊＊waitend＊＊＊＊＊＊＊＊＊＊" << std::endl;
	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標に到着＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag1.data = 1;
		robot1_pub.publish(arrive_flag1);
		robot1_test_pub.publish(goalpose);
	}
	else if(ac.getState() == actionlib::SimpleClientGoalState::ABORTED || goalstate == 4){
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標へのパス生成不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag1.data = 2;
		robot1_pub.publish(arrive_flag1);
		robot1_test_pub.publish(goalpose);
	}
	else{
		std::cout << "＊＊＊＊＊＊＊＊＊＊目標座標への移動不可＊＊＊＊＊＊＊＊＊＊" << std::endl;
		arrive_flag1.data = 3;
		robot1_pub.publish(arrive_flag1);
		robot1_test_pub.publish(goalpose);
		//return res.result;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot1_action");
	std::cout << "test" << std::endl;
	robot1_action R1A;

	std::cout << "test" << std::endl;

	R1A.param1.getParam("/multi_planning_server/robot1_action/map_frame1",R1A.frame_id);
	R1A.param1.getParam("/multi_planning_server/robot1_action/move_base_node1",R1A.move_base_node);

    while(ros::ok())
    {
		R1A.arrive_flag1.data = false;
        R1A.robot1_sub_queue.callOne();
		if(R1A.wait_flag)
		{
        	R1A.moveToGoal(R1A.x,R1A.y,R1A.frame_id,R1A.move_base_node);
		}
		R1A.wait_flag = false;
		R1A.rate.sleep();
    }
    return 0;
}
