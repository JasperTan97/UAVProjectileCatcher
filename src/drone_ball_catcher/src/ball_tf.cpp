#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
// Added tf2 headers to hold world frames
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

tf2_ros::Buffer tfBuffer; 
tf2_ros::TransformListener *tfListener;
ros::Publisher *pubber;


geometry_msgs::Point ball_location; // in map frame
geometry_msgs::Point ball_location_relative; // in drone frame
void ball_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    // cout << "sub run" << endl;
    // ball location is in camera_link frame now
    // set this relative location to ball_location_relative
    ball_location_relative.x = msg->point.x;
    ball_location_relative.y = msg->point.y;
    ball_location_relative.z = msg->point.z;
    // converting ball_location to map frame 
    //cout << "before tf: " << ball_location_relative.x << "," << ball_location_relative.y << "," << ball_location_relative.z << endl;
    try{
        //transformStamped = tfBuffer.lookupTransform("/map", "/camera_link", ros::Time(0));
        //tf2_geometry_msgs::do_transform(*msg, ball_location_map, transformStamped);
        geometry_msgs::PointStamped ball_location_map;

        tfBuffer.transform(*msg, ball_location_map, "map");
        //tfBuffer.lookupTransform("base_link", "camera_link",ros::Time(0));

        // ensure drone is not told to go underground
        /*
        if (ball_location_map.point.z <= 0){
            return;
        }*/

        ball_location.x = ball_location_map.point.x;
        ball_location.y = ball_location_map.point.y;
        ball_location.z = ball_location_map.point.z;

        //cout << "from tf: " << ball_location.x << "," << ball_location.y << "," << ball_location.z << endl;

        geometry_msgs::PointStamped::Ptr pub (new geometry_msgs::PointStamped);
        pub->header.frame_id = "map";
        pub->header.stamp = ros::Time::now();
        pub->point.x = ball_location.x;
        pub->point.y = ball_location.y;
        pub->point.z = ball_location.z;
        pubber->publish(pub);
    }
    catch (tf2::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
        //ros::Duration(1.0).sleep();
    }
}


int main(int argc, char **argv){
	ros::init(argc, argv, "ball_tf_node");
	ros::NodeHandle nh;

    // TF listener
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    // SUBSCRIBERS
    ros::Subscriber ball_locator_sub = nh.subscribe<geometry_msgs::PointStamped>
            ("ball_geom", 1000, ball_cb);

    // PUBLISHERS
    pubber = new ros::Publisher(nh.advertise<geometry_msgs::PointStamped>
    		("ball_tf",1000));


    ros::spin();
    //delete pubber;
}