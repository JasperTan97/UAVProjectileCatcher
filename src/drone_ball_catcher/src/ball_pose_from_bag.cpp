/* Used to extract and publish ball data from rosbag to ball_geom
*/

#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
ros::Publisher *pub;
ros::Publisher *printpub;

void callback(const gazebo_msgs::ModelStates::ConstPtr& msg){

    geometry_msgs::PointStamped ball_loc;
    geometry_msgs::PoseStamped print_ball_loc;

    ball_loc.header.frame_id = "map";
    ball_loc.header.stamp = ros::Time::now();
    ball_loc.point.x = msg->pose[2].position.x;
    ball_loc.point.y = msg->pose[2].position.y;
    ball_loc.point.z = msg->pose[2].position.z;

    pub->publish(ball_loc);

    print_ball_loc.header = ball_loc.header;
    print_ball_loc.pose.position = ball_loc.point;
    printpub->publish(print_ball_loc);

    // cout << msg->pose[2].position.x << endl;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_pose_node");
    ros::NodeHandle nh;
    ros::Subscriber ball_locator_sub = nh.subscribe<gazebo_msgs::ModelStates>
            ("gazebo/model_states_bag", 1000, callback);
    pub = new ros::Publisher(nh.advertise<geometry_msgs::PointStamped>("ball_geom", 1000));
    printpub = new ros::Publisher(nh.advertise<geometry_msgs::PoseStamped>("print_ball_geom", 1000));

    ros::spin();
}