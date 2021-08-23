#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <string>
#include <deque>
#include <vector>

using namespace std;

struct point_t{
	float x;
	float y;
	float z;
	double t;
};


// CONFIG
const int frame_cycle = 5; // max number of frames in queue for velocity estimation, a highher value reduces random error, but is less sensitive to acceleration changes
const double TIME_STEP = 0.01; // in seconds
const double MAX_TIME_PATH_PLANNING = 100; // in seconds
const double MAX_COUNT = 1000; //number of dt step
const double PI = 3.1415265;
const double MASS = 0.0027; // in KG
const double G = -9.81; //-9.781;// approx G in Singapore (in m/s^2)
const double DRAG_COEF = 0; //0.5; // for spheres at approx Reynold Number 5000
const double BALL_DIAMETER = 0.04; // in metres
const double AIR_DENSITY = 1.225; // in kg/m^3
string goal_type = "shortest_time";

// Global publisher pointer
ros::Publisher *setpoint_pubPtr;
ros::Publisher *printpoint_pubPtr;

// Create deque for the points and other global variables
deque<point_t> PoseData;
deque<point_t> PathPredicted;
geometry_msgs::PointStamped::Ptr goal (new geometry_msgs::PointStamped); // setpoint location for drone
geometry_msgs::PointStamped::Ptr prev_goal (new geometry_msgs::PointStamped); // in case ball is lost in sights, head to previous goal
geometry_msgs::PointStamped::Ptr prev_msg (new geometry_msgs::PointStamped); // in case ball is lost in sights, keep good messages
geometry_msgs::PoseStamped::Ptr print_goal (new geometry_msgs::PoseStamped);
geometry_msgs::PoseStamped current_location;
double t_0; // initial time
double mag_V, v_x, v_y, v_z; // velocities
double phi, theta; // angles (phi is Z to X axis and theta is X to Y axis)
double sum_t, sum_t_2, sum_x, sum_y, sum_z, sum_tx, sum_ty, sum_tz; // intermediate values for linear regression
double drag, s_x, s_y, s_z, a_x, a_y, a_z; // trajectory prediction intermediate variables
bool goal_uncheck = true; // bool variable check for iterative algorithm to stop 
float real_time_factor;
int starting_reads = 0; int max_starting_reads = 5;
float max_dist_step = 10;
bool first_accepted_sz = false;
float s_z_0 = 0;
float drone_speed = 5.0;
// Function Declarations
void callback(const geometry_msgs::PointStamped::ConstPtr& msg);
void add_point(const geometry_msgs::PointStamped&);
void compute_velocities();
void predict_trajectory(const geometry_msgs::PointStamped::ConstPtr& msg);
bool implement_plane_fixed_goal();

vector<point_t> pred_pose;
bool plane_fixed_path_planning(){
	if (s_x <= 0){
		// set the goal setpoint for the drone
		if(first_accepted_sz == true && abs(abs(s_z) - abs(s_z_0)) < 0.5){
			goal->point.x = 0;
			goal->point.y = s_y;
			goal->point.z = s_z;
		}
		first_accepted_sz = true;
		s_z_0 = s_z;
		return false;
	}
	else{
		return true;
	}
}

bool shortest_path_planning(){
	float min_shortest_path_planning = 100.0;
	float min_shortest_path_planning_x = 0;
	float min_shortest_path_planning_y = 0;
	float min_shortest_path_planning_z = 0;
	if (s_x <= 0){
		if(first_accepted_sz == true && abs(abs(s_z) - abs(s_z_0)) < 0.5){
			for (int i = 0; i < PathPredicted.size(); ++i){
				float dx = current_location.pose.position.x - PathPredicted[i].x;
				float dy = current_location.pose.position.y - PathPredicted[i].y;
				float dz = current_location.pose.position.z - PathPredicted[i].z;
				float min_d = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
				if (min_shortest_path_planning > min_d && PathPredicted[i].z > 0){
					min_shortest_path_planning = min_d;
					min_shortest_path_planning_x = PathPredicted[i].x;
					min_shortest_path_planning_y = PathPredicted[i].y;
					min_shortest_path_planning_z = PathPredicted[i].z;
				}
			}
			goal->point.x = min_shortest_path_planning_x;
			goal->point.y = min_shortest_path_planning_y;
			goal->point.z = min_shortest_path_planning_z;
			//cout << min_shortest_path_planning_x << "," << min_shortest_path_planning_y << "," <<  min_shortest_path_planning_z << "," <<  min_shortest_path_planning << endl;
		}
		first_accepted_sz = true;
		s_z_0 = s_z;
		return false;
	}
	else{
		return true;
	}
}

bool shortest_time_planning(){
	float min_shortest_time_planning_x = 0;
	float min_shortest_time_planning_y = 0;
	float min_shortest_time_planning_z = 0;
	if (s_x <= 0){
		if(first_accepted_sz == true && abs(abs(s_z) - abs(s_z_0)) < 0.5){
			float min_t = 100.0;
			for (int i = 0; i < PathPredicted.size(); ++i){
				float dx = current_location.pose.position.x - PathPredicted[i].x;
				float dy = current_location.pose.position.y - PathPredicted[i].y;
				float dz = current_location.pose.position.z - PathPredicted[i].z;
				float min_d = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));
				//cout << PathPredicted[i].t * drone_speed << "," << min_d << endl;

				if (PathPredicted[i].t * drone_speed > min_d && min_t > PathPredicted[i].t){
					min_shortest_time_planning_x = PathPredicted[i].x;
					min_shortest_time_planning_y = PathPredicted[i].y;
					min_shortest_time_planning_z = PathPredicted[i].z;
					min_t = PathPredicted[i].t;
				}
			}
			goal->point.x = min_shortest_time_planning_x;
			goal->point.y = min_shortest_time_planning_y;
			goal->point.z = min_shortest_time_planning_z;
			cout << min_shortest_time_planning_x << "," << min_shortest_time_planning_y << "," <<  min_shortest_time_planning_z << "," <<  min_t << endl;
		}
		first_accepted_sz = true;
		s_z_0 = s_z;
		return false;
	}
	else{
		return true;
	}
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
	current_location = *msg;
}

void compute_velocities(){

	// computes velocities using least squares linear regression
	// set intermediate values as 0.0;
	sum_t = sum_t_2 = sum_x = sum_y = sum_z = sum_tx = sum_ty = sum_tz = 0.0;
	// loop through PoseData to calculate intermediate values
	for (int i = 0; i < PoseData.size(); ++i){
		sum_t += PoseData[i].t;
		sum_t_2 += pow(PoseData[i].t, 2);
		sum_x += PoseData[i].x;
		sum_y += PoseData[i].y;
		sum_z += PoseData[i].z;
		sum_tx += (PoseData[i].t * PoseData[i].x);
		sum_ty += (PoseData[i].t * PoseData[i].y);
		sum_tz += (PoseData[i].t * PoseData[i].z);
	}
	// Apply least squares formula to calculate the slopes (velocity)
	v_x = ((PoseData.size() * sum_tx) - (sum_x * sum_t)) / ((PoseData.size() * sum_t_2) - pow(sum_t, 2));
	v_y = ((PoseData.size() * sum_ty) - (sum_y * sum_t)) / ((PoseData.size() * sum_t_2) - pow(sum_t, 2));
	v_z = ((PoseData.size() * sum_tz) - (sum_z * sum_t)) / ((PoseData.size() * sum_t_2) - pow(sum_t, 2));

}

void predict_trajectory(const geometry_msgs::PointStamped::ConstPtr& msg){
	// create vector for predicted trajectory
	vector<point_t> PredictedPose;
	PathPredicted.clear();
	// Calculate the maximum iteration count based on slowest speed approximation
	// double max_count = MAX_TIME_PATH_PLANNING / TIME_STEP;
	int count = 0;

	s_x = msg->point.x;
	s_y = msg->point.y;
	s_z = msg->point.z;

	//cout << "VEL" << v_x << ',' << v_y << ',' << v_z << endl;
	double current_time = 0;
	while(goal_uncheck && count <= MAX_COUNT){
		// calculate the velocity magnitude, vector angles, acclerations wrt point cloud frame
		mag_V = sqrt(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));
		phi = atan2(v_x, v_z);
		theta = atan2(v_y, v_x);	
		drag = 0.5 * AIR_DENSITY * DRAG_COEF * pow(mag_V, 2) * PI * pow(BALL_DIAMETER / 2, 2);
		a_y = - (drag / MASS) * sin(phi) * cos(theta);
		a_z = (drag / MASS) * sin(phi) * sin(theta) + G;
		a_x = - (drag / MASS) * cos(phi) * cos(theta);
		// caclulate the displacements in x,y,z direction after one time step using the equation of motion (assuming constant velocity throughout the time step)
		s_x = s_x + v_x * TIME_STEP + 0.5 * a_x * pow(TIME_STEP, 2);
		s_y = s_y + v_y * TIME_STEP + 0.5 * a_y * pow(TIME_STEP, 2);
		s_z = s_z + v_z * TIME_STEP + 0.5 * a_z * pow(TIME_STEP, 2);
		// update velocities in 3 directions after one time step (assuming constant acceleration)
		v_x = v_x + a_x * TIME_STEP;
		v_y = v_y + a_y * TIME_STEP;
		v_z = v_z + a_z * TIME_STEP;
		// add to deque of points
		current_time = current_time + TIME_STEP;
		point_t temp_point_t_0;
		temp_point_t_0.x = s_x;
		temp_point_t_0.y = s_y;
		temp_point_t_0.z = s_z;
		temp_point_t_0.t = current_time;
		PathPredicted.push_front(temp_point_t_0);
		// check to see if iterative steps should cease
		if (goal_type == "plane_fixed"){
			goal_uncheck = plane_fixed_path_planning();
		}
		if (goal_type == "shortest_path"){
			goal_uncheck = shortest_path_planning();
		}
		if (goal_type == "shortest_time"){
			goal_uncheck = shortest_time_planning();
		}
		count++;
	}
}

void add_point(const geometry_msgs::PointStamped& pt){
	// Add point to PoseData deque
	point_t temp_point_t;
	temp_point_t.x = pt.point.x;
	temp_point_t.y = pt.point.y;
	temp_point_t.z = pt.point.z;

	// use ROS time to input time
	if (PoseData.size() == 0){
		t_0 = ros::Time::now().toSec();
		temp_point_t.t = 0.0;
	}
	else{
		temp_point_t.t = (ros::Time::now().toSec() - t_0)*real_time_factor;
	}
	// push to the front

	PoseData.push_front(temp_point_t);
}


void callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	
	/*
	if(starting_reads >= max_starting_reads && (abs(msg->point.x) > max_dist_step || 
			abs(msg->point.y) > max_dist_step || abs(msg->point.z) > max_dist_step)){
		msg->point.x = NAN;
		msg->point.y = NAN;
		msg->point.z = NAN;
	}*/
	
	if (PoseData.size() < frame_cycle){
		add_point(*msg);
		// ensure that at least two data points are present;
		if (PoseData.size() < 5){
			return;
		}
	}
	else{
		// remove the first point in the frame cycle and add the new point (like a queue)
		PoseData.pop_back();
		add_point(*msg);
	}
	// set default goal->x setpoint to NAN first (math.h macro)
	goal->point.x = goal->point.y = goal->point.z = NAN;
	// calculate the velocities in 3 directions and get the Magnitude of V and its angles
	compute_velocities();
	// make sure ball is moving towards drone (0.1m/s clearance given)
	if (v_x > -0.1){
		return;
	}
	// predict the trajectory of ball and conduct drone path planning and extract goal setpoint
	predict_trajectory(msg);
	// publish goal setpoint
	if(true || !isnan(goal->point.x)){
		goal->header.frame_id = msg->header.frame_id;
		goal->header.stamp = ros::Time::now();
		setpoint_pubPtr->publish(goal);
		print_goal->header = goal->header;
		print_goal->pose.position = goal->point;
		printpoint_pubPtr->publish(print_goal);

	*prev_msg = *msg;
	*prev_goal = *goal;
	}
	
	// reset goal_uncheck to true
	goal_uncheck = true;

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ball_trajectory_node");
	ros::NodeHandle nh;

    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1000, pose_cb);
	ros::Subscriber sub = nh.subscribe<geometry_msgs::PointStamped>("ball_tf", 10000, callback);
	setpoint_pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::PointStamped>("predicted_setpoint", 10000));
	printpoint_pubPtr = new ros::Publisher(nh.advertise<geometry_msgs::PoseStamped>("print_predicted_setpoint", 10000));


	nh.getParam("real_time_factor", real_time_factor);

	ros::spin();

	delete setpoint_pubPtr;
}