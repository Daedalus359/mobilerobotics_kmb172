//kevin bradner's modification of...
//path_client:
// illustrates how to send a request to the path_service service

#include <ros/ros.h>
#include <example_ros_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<example_ros_service::PathSrv>("path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    example_ros_service::PathSrv path_srv;
	
    double pi = 3.142;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;

    //initial pose command
    pose.position.x = 0.0; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    
    // face north
    quat = convertPlanarPhi2Quaternion(pi / 2.0); // get a quaternion corresponding to this heading
    pose_stamped.pose.orientation = quat;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go north 3
    quat = convertPlanarPhi2Quaternion(pi / 2.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 3.0;
    pose_stamped.pose.position.x = 0.0;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go east 6.5
    quat = convertPlanarPhi2Quaternion(0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 0.0;
    pose_stamped.pose.position.x = 6.5;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go north 8.15
    quat = convertPlanarPhi2Quaternion(pi / 2.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 8.15;
    pose_stamped.pose.position.x = 0.0;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go west 3
    quat = convertPlanarPhi2Quaternion(pi);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 0.0;
    pose_stamped.pose.position.x = -3.0;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go north 1.1
    quat = convertPlanarPhi2Quaternion(pi / 2.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 1.1;
    pose_stamped.pose.position.x = 0.0;
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    // go west 3
    quat = convertPlanarPhi2Quaternion(pi);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 0.0;
    pose_stamped.pose.position.x = -3.0;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    //send the list of poses off for execution by the server
    client.call(path_srv);

    return 0;
}
