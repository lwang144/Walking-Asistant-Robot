/*
 * send_goal.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: unicorn
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
/*move_base_msgs::MoveBaseAction
 move_base在world中的目标
*/ 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;


int main(int argc, char** argv) {
	ros::init(argc, argv, "send_goals_node");
	/*
	// create the action client
	// true causes the client to spin its own thread
	//don't need ros::spin()
	创建action客户端，参数1：action名，参数2：true，不需要手动调用ros::spin()，会在它的线程中自动调用。
	*/
	MoveBaseClient ac("move_base", true);
	// Wait 60 seconds for the action server to become available
	ROS_INFO("Waiting for the move_base action server");
	ac.waitForServer(ros::Duration(60));
	ROS_INFO("Connected to move base server");
	// Send a goal to move_base
	//目标的属性设置
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	goal.target_pose.pose.position.x = 5.60237;
	goal.target_pose.pose.position.y = -1.8963;
	goal.target_pose.pose.orientation.w = 1;
	ROS_INFO("");
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	// Wait for the action to return
	ac.waitForResult();
	if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("You have reached the goal!");
	else
		ROS_INFO("The base failed for some reason");
	return 0;
}
