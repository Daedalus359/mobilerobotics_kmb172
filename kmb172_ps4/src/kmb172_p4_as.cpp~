//Kevin Bradner's action server for mobile robotics ps4
//takes and executes a set of open loop control commands

//based heavily on the example_action_server from learning_ros

#include<ros/ros.h>
#include<actionlib/server/simple_action_server.h>
#include<kmb172_ps4/p4msg.h>
#include<std_msgs>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>//pow

class ps4ActionServer{
private:

	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<kmb172_p4::p4msgAction> as_;
	kmb172_p4::p4msgGoal goal_;
	kmb172_p4::p4msgResult result_;
	kmb172_p4::p4msgFeedback feedback_;//not sure how to actually use this or if I need to

public:
	ps4ActionServer();
	
	~ps4ActionServer(void) {
	}

	void executeCB(const actionlib::SimpleActionServer<kmb172_p4::p4msgAction>::GoalConstPtr& goal);
};//what's up with the syntax on this line?

ps4ActionServer::ps4ActionServer() ://TODO: correct for this package?
	as_(nh_, "p4_action", boost::bind(&ps4ActionServer::executeCB, this, _1),false)
{
	ROS_INFO("in constructor of action server");
	//TODO: do I need any initializations here?
	as_.start();
}

void ps4ActionServer::executeCB(const actionlib::SimpleActionServer<kmb172_ps4::p4msgAction>::GoalConstPtr & goal){
	//start moving I guess
	//do I need to check for cancellations around here?
	
}
