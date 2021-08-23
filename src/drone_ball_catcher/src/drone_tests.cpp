/**
Drone tests:
Scenario A: Drone motion: Translational; Ball motion: Fixed; Drone path planning: Cat and mouse
Scenario B: Drone motion: Translational; Ball motion: Linear; Drone path planning: Cat and mouse
Scenario C: Drone motion: Translational and Rotational; Ball motion: Linear; Drone path planning: Cat and mouse
Scenario D: Drone motion: Translational and Rotational; Ball motion: Parabolic;  Drone path planning: Shortest path
Scenario E: Drone motion: Translational and Rotational; Ball motion: Parabolic;  Drone path planning: Fastest path
 */

#include <ros/ros.h>
#include <math.h>
#include <string.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


using namespace std; 

bool look_for_ball = false;
bool move_drone = false;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::Pose current_location;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	current_location.position.x = msg->pose.position.x;
    current_location.position.y = msg->pose.position.y;
    current_location.position.z = msg->pose.position.z;
    current_location.orientation.w = msg->pose.orientation.w;
    current_location.orientation.x = msg->pose.orientation.x;
    current_location.orientation.y = msg->pose.orientation.y;
    current_location.orientation.z = msg->pose.orientation.z;

}

geometry_msgs::Point ball_location;
void ball_cb(const geometry_msgs::Point::ConstPtr& msg)
{
    look_for_ball = true;
    ball_location.x = msg->x;
    ball_location.y = msg->y;
    ball_location.z = msg->z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drone_test_1");
    ros::NodeHandle nh;

    // SUBSCRIBERS
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
			("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber ball_locator_sub = nh.subscribe<geometry_msgs::Point>
            ("ball_geom", 10, ball_cb);

    // PUBLISHERS
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);

    // SERVICES
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    // TRANSFORM FRAME
    tf2_ros::Buffer tfBuffer;
  	tf2_ros::TransformListener tfListener(tfBuffer);

    // ************************************************ START OF DRONE SETUP ************************************************

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ROS_INFO("Connecting to drone");
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected!");


	geometry_msgs::TwistStamped msg;
	msg.twist.linear.x = 0;
	msg.twist.linear.y = 0;
	msg.twist.linear.z = 0;


    //send a few setpoints before starting
    for(int i = 20; ros::ok() && i > 0; --i){
        vel_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Setting mode to OFFBOARD");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    ROS_INFO("Arming drone");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    geometry_msgs::PoseStamped initial_location;
    initial_location.pose.position.x = 0;
    initial_location.pose.position.y = 0;
    initial_location.pose.position.z = 2;
    
    initial_location.pose.orientation.x = 0.01;
    initial_location.pose.orientation.y = 0.01;
    initial_location.pose.orientation.z = 0.01;
    initial_location.pose.orientation.w = 0.99;
    geometry_msgs::PoseStamped target_location;

    target_location.header.frame_id = "base_link";
    int min_readings = 5;
    int reading_count = 0;

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // ************************************************ END OF DRONE SETUP************************************************
        // add code for drone movement here

        // extract scenario parameter from config/scenario_type.yaml
        std::string scenario;
        nh.getParam("scenario", scenario);
        // scenario A and B has similar drone behaviours, but different ball behaviour
        if(scenario == "A" || scenario == "B"){
        	cout << "Scenario A/B" << endl;
	        if (!look_for_ball || (isnan(ball_location.x) || isnan(ball_location.y) || isnan(ball_location.z))){
	        }
	        else if (!move_drone){
	            if (reading_count < min_readings){
	                reading_count++;
	            }
	            else{
	            target_location.pose.position.x = current_location.position.x + ball_location.z;
	            target_location.pose.position.y = current_location.position.y - ball_location.x;
	            target_location.pose.position.z = current_location.position.z - ball_location.y;
	            look_for_ball = false;
	            move_drone = true;
	            //ROS_INFO("Ball Detected");
	            //cout << "Moving to location " << target_location.pose.position.x << "," << target_location.pose.position.y << "," << target_location.pose.position.z << endl;
	            }
	        }
	        

	        if (move_drone){
	            local_pos_pub.publish(target_location);
	        }
	        else{
	            local_pos_pub.publish(initial_location);
	        }

	        ros::spinOnce();
	        rate.sleep();
	    }


	    //scenario C includes yawing
	    else if(scenario == "C"){
	        if (!look_for_ball || (isnan(ball_location.x) || isnan(ball_location.y) || isnan(ball_location.z))){
	        }
	        else if (!move_drone){
	        	cout << "I found the ball and I am moving" << endl;
	        	//transform frame from camera to base link 
			    geometry_msgs::TransformStamped transformCamtoBaseStamped;
			    try{
			      transformCamtoBaseStamped = tfBuffer.lookupTransform("base_link", "camera_link",
			                               ros::Time(0));
			    }
			    catch (tf2::TransformException &ex) {
			      //ROS_WARN("%s",ex.what());
			      ros::Duration(1.0).sleep();
			      continue;
			    }

			    //use transformCamtoBaseStamped to convert ball location from camera frame to base frame
			    geometry_msgs::Point transformed_ball_location;
			    tf2::doTransform(ball_location, transformed_ball_location, transformCamtoBaseStamped);
			    cout << "Original: " << ball_location.x << "," << ball_location.y << "," << ball_location.z << endl;
			    cout << "Transformed: " << transformed_ball_location.x << "," << transformed_ball_location.y << "," << transformed_ball_location.z << endl;

				if (reading_count < min_readings){
	                reading_count++;
	                //cout << "reading_count: " << reading_count << endl;
	            }
	            else{
	            	float theta_max = 0.7;
	            	float theta = atan(-1 * transformed_ball_location.y / transformed_ball_location.x);
	            	cout << theta << endl;
	            	if(abs(theta) > theta_max){
	            		// rotate back
	            		tf2::Quaternion toRotate, newRotate, currentRotate;
	            		toRotate.setRPY( 0, 0, (theta / 2));
	            		tf2::convert(current_location.orientation, currentRotate);
	            		newRotate = toRotate*currentRotate;
	            		tf2::convert(newRotate, target_location.pose.orientation);
	         			look_for_ball = true;
			            move_drone = true;
			            cout << "Rotating... " << endl;
	            	} else{
			            target_location.pose.position.x = transformed_ball_location.x;
			            target_location.pose.position.y = transformed_ball_location.y;
			            target_location.pose.position.z = transformed_ball_location.z;
			            look_for_ball = true;
			            move_drone = true;
			            ROS_INFO("Ball Detected");
			            cout << "Moving to location " << target_location.pose.position.x << "," << target_location.pose.position.y << "," << target_location.pose.position.z << endl;
	            	}
	            }
	        }
	        if (move_drone){
	            local_pos_pub.publish(initial_location);
	        }
	        else{
	            local_pos_pub.publish(initial_location);
	        }
	        cout << "Target location: " << target_location.pose.position.x << "," << target_location.pose.position.y << "," << target_location.pose.position.z << "," << target_location.pose.orientation.w <<"," << target_location.pose.orientation.x <<"," << target_location.pose.orientation.y << "," << target_location.pose.orientation.z << endl;
	        ros::spinOnce();
	        rate.sleep();
	    }
	}

    return 0;
}
