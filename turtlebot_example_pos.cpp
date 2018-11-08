//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cppos 
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <stdlib.h>

ros::Publisher pose_publisher;
ros::Publisher pose_array_publisher;
ros::Publisher marker_pub;


struct quat{
	double w;
	double x;
	double y;
	double z;

};

double ips_x;
double ips_y;
double ips_yaw;
double odom_x;
double odom_y;
double odom_yaw;
double odom_vx;
double odom_vy;
double odom_az;
time_t timer;
tf::Matrix3x3 mat;
tf2::Quaternion myQ;
quat q;

short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x;
    ips_y = msg.pose[i].position.y;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);

    if(ips_yaw<0){ //need to map
 	ips_yaw += 2*M_PI;
    }

}

void odom_callback(const nav_msgs::Odometry &msg){
    
    odom_x = msg.pose.pose.position.x;
    odom_y = msg.pose.pose.position.y;
    odom_yaw = tf::getYaw(msg.pose.pose.orientation);

    odom_vx = msg.twist.twist.linear.x;
    odom_vy = msg.twist.twist.linear.y;
    odom_az = msg.twist.twist.angular.z;
    if(odom_yaw<0){ //need to map
 	odom_yaw += 2*M_PI;
    }
}

quat e2q(double pitch, double roll, double yaw){

	double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    quat q_temp;
    q_temp.w = cy * cr * cp + sy * sr * sp;
    q_temp.x = cy * sr * cp - sy * cr * sp;
    q_temp.y = cy * cr * sp + sy * sr * cp;
    q_temp.z = sy * cr * cp - cy * sr * sp;
    return q_temp;

}
//Callback function for the Position topic (LIVE)
/*
void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{

	ips_x X = msg.pose.pose.position.x; // Robot X psotition
	ips_y Y = msg.pose.pose.position.y; // Robot Y psotition
	ips_yaw = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw
	ROS_DEBUG("pose_callback X: %f Y: %f Yaw: %f", X, Y, Yaw);
}*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<int> &x, std::vector<int> &y) {

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int dx2 = x1 - x0;
    int dy2 = y1 - y0;

    const bool s = abs(dy) > abs(dx);

    if (s) {
        int dx2 = dx;
        dx = dy;
        dy = dx2;
    }

    int inc1 = 2 * dy;
    int d = inc1 - dx;
    int inc2 = d - dx;

    x.push_back(x0);
    y.push_back(y0);

    while (x0 != x1 || y0 != y1) {
        if (s)
            y0 += sgn(dy2);
        else
            x0 += sgn(dx2);
        if (d < 0)
            d += inc1;
        else {
            d += inc2;
            if (s)
                x0 += sgn(dx2);
            else
                y0 += sgn(dy2);
        }

        //Add point to vector
        x.push_back(x0);
        y.push_back(y0);
    }
}


float normpdf(float meas, float mean, float sigma){
    return ( 1 / (sigma*sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (meas-mean) / sigma,2));
}


int P = 100;  //Number of particles
Eigen::MatrixXf state_previous(3,1);
Eigen::MatrixXf state_current(3,1);
Eigen::MatrixXf state_update(3,1);
Eigen::MatrixXf meas(3,1);
Eigen::MatrixXf e(3,1);
Eigen::MatrixXf d(3,1);

Eigen::MatrixXf X(3,P);
Eigen::MatrixXf w(3,P);
Eigen::MatrixXf W(3,P);
//float W;
std::default_random_engine generator;
float R = 0.1;
float Q = 0.1;

Eigen::MatrixXf Xp(3,P);
float V;
float U;
float mean_x;
float mean_y;
float mean_z;
float dt = 0.05;
float seed;

geometry_msgs::Pose pose; 
geometry_msgs::PoseArray pose_array;


int main(int argc, char **argv) {
    std::ofstream logFile;
    logFile.open("/home/colin/ParticleFilter.txt");


    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber odom_sub = n.subscribe("/odom",1,odom_callback);

    std::normal_distribution<double> distribution_e(0,R);
    std::normal_distribution<double> distribution_d(0,Q);
    std::uniform_real_distribution<double> distribution_i(-1,1);
    std::uniform_real_distribution<double> rand(0,1);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::Pose>("/pose", 1, true);
    pose_array_publisher = n.advertise<geometry_msgs::PoseArray>("/pose_array",1,true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);


    //Velocity control variable
    geometry_msgs::Twist vel;

    for(int i=0;i<P;i++){
        X(0,i) = distribution_i(generator);
        X(1,i) = distribution_i(generator);
        X(2,i) = distribution_i(generator);
    }

    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate

    //Initialize state


    state_previous(0) = 0;
    state_previous(1) = 0;
    state_previous(2) = 0;
    float mean_x = 0;
    float var_x = 0;

    while (ros::ok()) {

	        //Main loop code goes here:
		    V = 0.2;
		    U = 0.2;
	        vel.linear.x = V;  // set linear speed
	        vel.angular.z = U; // set angular speed
	        //vel.linear.y = V;
	        velocity_publisher.publish(vel); // Publish the command velocity

		//Motion model update

	        e(0) = distribution_e(generator);
		    e(1) = distribution_e(generator);
		    e(2) = distribution_e(generator);

	        state_current(0) = state_previous(0) + odom_vx*cos(ips_yaw)*0.05;
	        state_current(1) = state_previous(1) + odom_vx*sin(ips_yaw)*0.05;
	        state_current(2) = state_previous(2) + odom_az*0.05;
	        if(state_current(2)>2*M_PI){
	        	state_current(2)=state_current(2)-2*M_PI;
	        }

		//Measurement update

		    d(0) = distribution_d(generator);
		    d(1) = distribution_d(generator);
		    d(2) = distribution_d(generator);
		
	        meas(0) = ips_x + d(0);
		    meas(1) = ips_y + d(1);
		    meas(2) = ips_yaw + d(2);		
		    int counter_x = 0;
		    float sum_x = 0;
			//Particle filter
		    for(int i=0;i<P;i++){
		        e(0) = distribution_e(generator); //Updates the state space model
		        e(1) = distribution_e(generator);
		        e(2) = distribution_e(generator);

		        Xp(0,i) = X(0,i) + odom_vx*cos(ips_yaw)*0.05 + e(0);
		        Xp(1,i) = X(1,i) + odom_vx*sin(ips_yaw)*0.05 + e(1);
		        Xp(2,i) = X(2,i) + odom_az*0.05 + e(2); 
		        if(Xp(2,i)>2*M_PI){
	        		Xp(2,i)=Xp(2,i)-2*M_PI;
	        	}
		        w(0,i) = normpdf(meas(0),Xp(0,i),Q); //Generates the particle distribution function
		        w(1,i) = normpdf(meas(1),Xp(1,i),Q);
		        w(2,i) = normpdf(meas(2),Xp(2,i),Q);
		        if(i==0){ //cumulative sum
		        	W(0,i) = w(0,i);
		        	W(1,i) = w(1,i);
		        	W(2,i) = w(2,i);
		        }
		        else{
		        	W(0,i) = W(0,i-1) + w(0,i);
		        	W(1,i) = W(1,i-1) + w(1,i);
		        	W(2,i) = W(2,i-1) + w(2,i);
		        }
		    }
		    for(int i=0;i<P;i++){ //Take particle that is correct for our range and add it to the particle set
		    	seed = rand(generator);
		    	//ROS_INFO("seed: %f", seed);
		    	for(int j=0;j<P;j++){
		    		//seed = rand(generator);
		    		if(W(0,j)>W(0,P-1)*seed){
		    			X(0,i) = Xp(0,j);
						break;
		    		}
		    	}
		    	//ROS_INFO("Continuing...");
		    	for(int j=0;j<P;j++){
		    		//seed = rand(generator);
		    	
                    if(W(1,j)>W(1,P-1)*seed){
        
                    	X(1,i) = Xp(1,j);
                    	break;
                    }
		    	}
		    	for(int j=0;j<P;j++){
		    		//seed = rand(generator);
		    	
                    if(W(2,j)>W(2,P-1)*seed){
                	
                    	X(2,i) = Xp(2,j);
                    	break;
                    }
		    	}
		    }

			pose_array.poses.clear();
			for(int i=0;i<P;i++){
				pose.position.x = X(0,i);
				pose.position.y = X(1,i);
				

				q = e2q(0,0,X(2,i));
				pose.orientation.x = q.x;
				pose.orientation.y = q.y;
				pose.orientation.z = q.z;
				pose.orientation.w = q.w;

			    pose_array.poses.push_back(pose);			    
			}


			//ROS_INFO("Mean: %f, Var: %f, Sum: %f", mean_x, var_x, sum_x);
	float xSum = 0;
	float ySum = 0;
	for(int i=0;i<P;i++){
		xSum += X(0,i);
		ySum += X(1,i);
	}
	float xMean = xSum/P;
	float yMean = ySum/P;



	//logFile << "X Position: " << ips_x << "\n";
	//logFile << "Mean        " << mean << "\n\n";

	logFile << ips_x << ","<<xMean<<"\n";

	pose_array.header.frame_id = "/base_link";
	pose_array_publisher.publish(pose_array);
	ROS_INFO("mean: %f, actual: %f, current state: %f",yMean,ips_x,state_current(0));
	state_previous(0) = state_current(0);
	state_previous(1) = state_current(1);
	state_previous(2) = state_current(2);

    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages
   }

    return 0;
}
