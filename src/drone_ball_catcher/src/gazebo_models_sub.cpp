/* Used to extract and publish ball data from rosbag to ball_geom
*/

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;

bool first_msg = false;
ofstream myfile; 

void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){

    if(!first_msg){
        myfile.open ("Drone_ball_error.txt");
        myfile << "dx dy dz theta phi RMSE \n";
        first_msg = true;
    }

    geometry_msgs::PointStamped ball_loc;
    geometry_msgs::PointStamped drone_loc;
    geometry_msgs::PoseStamped print_ball_loc;

    float dx, dy, dz, theta, phi, RMSE; 

    drone_loc.header.frame_id = "map";
    drone_loc.header.stamp = ros::Time::now();
    drone_loc.point.x = msg->pose[1].position.x;
    drone_loc.point.y = msg->pose[1].position.y;
    drone_loc.point.z = msg->pose[1].position.z;

    ball_loc.header.frame_id = "map";
    ball_loc.header.stamp = ros::Time::now();
    ball_loc.point.x = msg->pose[2].position.x;
    ball_loc.point.y = msg->pose[2].position.y;
    ball_loc.point.z = msg->pose[2].position.z;

    dx = ball_loc.point.x - drone_loc.point.x;
    dy = ball_loc.point.y - drone_loc.point.y;
    dz = ball_loc.point.z - drone_loc.point.z;
    theta = acos(dz/sqrt(dx*dx + dy*dy));
    phi = atan(dy/dx);
    RMSE = sqrt(dx*dx + dy*dy + dz*dz);

    myfile << dx << " " << dy << " " << dz << " " << theta << " " << phi << " " << RMSE << "\n";

}

int main(int argc, char **argv){
    ros::init(argc, argv, "droneball_pose_node");
    ros::NodeHandle nh;
    ros::Subscriber ball_locator_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("/gazebo/model_states", 1000, callback);;    
    ros::spin();
}