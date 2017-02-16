//Kevin Bradner's trajectory generation code for the ariac competition qualifier
//TODO: make a dummy client that makes up poses to request for trajectory creation

//general c++ utilities
#include <algorithm>
#include <vector>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
//TODO: add include statement for message type of the request receipt interface here

sensor_msgs::JointState current_joint_state_;

int main(int argc, char ** argv){
    //set up a ros node
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    //TODO: add a service to the node (is this the correct type of interface?)
    /*
    ros::ServiceServer service = nh.advertiseService("trajectory_service", callback);
    ROS_INFO("trajectory service ready");
    */
	
    //add an action client for sending trajectories off to the action server that executes them
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> action_client("/ariac/arm/follow_joint_trajectory", true);

    ros::Subscriber my_subscriber_object= n.subscribe("/ariac/joint_states",1,joint_statesCallback);

    bool server_exists = action_client.waitForServer(ros::Duration(1.0));
    while (!server_exists) {
        ROS_WARN("waiting on arm server...");
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        server_exists = action_client.waitForServer(ros::Duration(1.0));
    }

    ROS_INFO("Sending Trajectory to server for execution");//send the supplied trajectory off to the action server for execution
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;
    action_client.sendGoal(goal, &armDoneCb);

    ros::spin();
    return 0;
}

/*TODO:: add this back in
bool callback(example_ros_service::ExampleServiceMsgRequest& request, example_ros_service::ExampleServiceMsgResponse& response){
    //what kinds of requests will I be getting here?
}
*/

void append_traj(trajectory_msgs::JointTrajectoryPoint jspace_pose, trajectory_msgs::JointTrajectory &traj_msg){
    traj_msgs.points = traj_msgs.points.push_back(jspace_pose);//add the newly resquested pose
}

void compute_traj_to_grasp_pose(int sector_code, trajectory_msgs::JointTrajectoryPoint jspace_pose, trajectory_msgs::JointTrajectory &trajectory){

    //start by defining the most easily accessible tuck pose from current position on rail
    double elbow_joint_position = 2.89;
    double linear_arm_actuator_position = current_joint_state_.position[1];
    double shoulder_lift_position = -1.51;
    double shoulder_pan_position = 3.02;
    double wrist_1_position = 3.02;
    double wrist_2_position = -1.63;
    double wrist_3_position = current_joint_state_.position[6];

    //initialize nextPose with the tuck position joint angles needed
    trajectory_msgs::JointTrajectoryPoint nextPose;
    nextPose.positions.push_back(elbow_joint_position);
    nextPose.positions.push_back(linear_arm_actuator_position);
    nextPose.positions.push_back(shoulder_lift_position);
    nextPose.positions.push_back(shoulder_pan_position);
    nextPose.positions.push_back(wrist_1_position);
    nextPose.positions.push_back(wrist_2_position);
    nextPose.positions.push_back(wrist_3_position);
 
    //append the tuck pose to the trajectory
    append_traj(nextPose, &trajectory);

    //calculate the rail position for the next move
    linear_arm_actuator_position = get_rail_position(sector_code);

    //append the rail positioned tuck pose to the trajectory
    nextPose.positions[0] = elbow_joint_position;
    nextPose.positions[1] = linear_arm_actuator_position);
    nextPose.positions[2] = shoulder_lift_position);
    nextPose.positions[3] = shoulder_pan_position);
    nextPose.positions[4] = wrist_1_position);
    nextPose.positions[5] = wrist_2_position);
    nextPose.positions[6] = wrist_3_position);
    append_traj(nextPose, &trajectory);

    //an approach pose over the area of interest
    if(sector_code < 5){//in one of the bins closer to the rail
	elbow_joint_position = 2.14;
	shoulder_lift_position = -0.75;
	wrist_1_position = 3.39;
    }
    else{//in one of the bins further from the rail
	elbow_joint_position = 0.63;
	shoulder_lift_position = -0.13;
	wrist_1_position = 4.27;
    }

    //append the pre-grasp/place pose to the trajectory
    nextPose.positions[0] = elbow_joint_position;
    nextPose.positions[1] = linear_arm_actuator_position);
    nextPose.positions[2] = shoulder_lift_position);
    nextPose.positions[3] = shoulder_pan_position);
    nextPose.positions[4] = wrist_1_position);
    nextPose.positions[5] = wrist_2_position);
    nextPose.positions[6] = wrist_3_position);
    append_traj(nextPose, &trajectory);

    //a grasp/place pose
    elbow_joint_position = 0.75;

    //append the grasp/place pose to the trajectory
    nextPose.positions[0] = elbow_joint_position;
    nextPose.positions[1] = linear_arm_actuator_position);
    nextPose.positions[2] = shoulder_lift_position);
    nextPose.positions[3] = shoulder_pan_position);
    nextPose.positions[4] = wrist_1_position);
    nextPose.positions[5] = wrist_2_position);
    nextPose.positions[6] = wrist_3_position);
    append_traj(nextPose, &trajectory);

    //send the completed trajectory off for execution
    execute_trajectory(trajectory);
}

//TODO: not sure which sector is which, need to edit this one accordingly
double get_rail_position(int sector_code){
    if((sector_code == 1) || (sector_code == 8)){
        return -1.34;
    }
    else if((sector_code == 2) || (sector_code == 7)){
        return -0.46;
    }
    else if((sector_code == 3) || (sector_code == 6)){
        return 0.21;
    }
    else if((sector_code == 4) || (sector_code == 5)){
        return 0.97;
    }
    else{
    ROS_INFO("inconsistent sector code, defaulting to sector 1");
        return -1.34;
    }
}

void joint_statesCallback(const sensor_msgs::JointState& joint_state_holder){
    current_joint_state_ = joint_state_holder;
}
