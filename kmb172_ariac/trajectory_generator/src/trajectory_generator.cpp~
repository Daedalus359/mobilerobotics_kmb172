//Kevin Bradner's trajectory generation code for the ariac competition qualifier

//general c++ utilities
#include <algorithm>
#include <vector>

//general ROS utilities
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

//ARIAC utilities
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
//TODO: add any service and actionserver message definitions here

int main(int argc, char ** argv){
    //set up a ros node
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    //add a service to the node (is this the correct type of interface?)
    ros::ServiceServer service = nh.advertiseService("trajectory_service", callback);
    ROS_INFO("trajectory service ready");
	
    //add an action client for sending trajectories off to the action server that executes them
    //TODO: make this line match the actual type of the action server
    actionlib::SimpleActionClient<example_action_server::demoAction> action_client("example_action", true);

    ros::spin();
    return 0;
}

bool callback(example_ros_service::ExampleServiceMsgRequest& request, example_ros_service::ExampleServiceMsgResponse& response){
    //what kinds of requests will I be getting here?
}

//TODO: verify that these are the correct data types for this function
void append_traj(sensor_msgs::JointState jspace_pose, trajectory_msgs::JointTrajectory &traj_msg){
    //modifies a trajectory 
    //TODO: figure out what these data types are and how to support them
    //TODO: figure out how to append a specified joint space pose to the trajectory
    //TODO: should I send to the action server for execution in this one?
}

//TODO: verify that these are the correct data types for this function
void compute_traj_to_grasp_pose(int sector_code, sensor_msgs::JointState grasp_pose, trajectory_msgs::JointTrajectory &trajectory){
    //given a sector and a desired gripper pose for part grasp/placement, assemble into the trajectory parameter the following sequence of poses:
	//a pose for tucking the arm
	//a pose to move along the rail
	//any intermediate pose as needed
	//an approach pose over the area of interest
	//a grasp/place pose
    //TODO: also send to the action server for execution
}

void exectute_trajectory(trajectory_msgs::JointTrajectory &trajectory){
    //send the supplied trajectory off to the action server for execution
}
