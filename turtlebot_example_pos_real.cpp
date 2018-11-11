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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <random>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <stdlib.h>

ros::Publisher pose_real_publisher;
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
double odom_vx;
double odom_vy;
double odom_az;
double odom_x;
double odom_y;
double odom_yaw;
std::vector<double> Q_row;


short sgn(int x) { return x >= 0 ? 1 : -1; }

//Callback function for the Position topic (SIMULATION)
/*void pose_callback(const gazebo_msgs::ModelStates &msg) {

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

}*/

void odom_callback(const nav_msgs::Odometry &msg){
    odom_vx = msg.twist.twist.linear.x;
    odom_vy = msg.twist.twist.linear.y;
    odom_az = msg.twist.twist.angular.z;

    odom_x = msg.pose.pose.position.x;
    odom_y = msg.pose.pose.position.y;
    odom_yaw = tf::getYaw(msg.pose.pose.orientation);

    //ROS_INFO("odom_callback vx: %f vy: %f az: %f", odom_vx, odom_vy, odom_az);

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
    if(q_temp.w>1){
    	q_temp.w = 1 - q_temp.w;
    }
    q_temp.x = cy * sr * cp - sy * cr * sp;
    q_temp.y = cy * cr * sp + sy * sr * cp;
    q_temp.z = sy * cr * cp - cy * sr * sp;
    return q_temp;

}

float mean(Eigen::MatrixXf m, int index){
	float sum = 0;
	for(int i=0;i<m.rows();i++){
		sum += m(index, i);
        //ROS_INFO("sum: %f",sum);
	}
    //ROS_INFO("x size: %ld",m.rows());
	return sum/m.rows();
}
//Callback function for the Position topic (LIVE)

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	ips_x = msg.pose.pose.position.x ; // Robot X position
	ips_y = msg.pose.pose.position.y; // Robot Y position
	ips_yaw = tf::getYaw(msg.pose.pose.orientation)+M_PI; // Robot Yaw

    if(ips_yaw<0){ //need to map
        ips_yaw += 2*M_PI;
    }

	//ROS_INFO("pose_callback X: %f Y: %f Yaw: %f", ips_x, ips_y, ips_yaw);
}

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid &msg) {
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

float normpdf(float meas, float mean, float sigma){
    return ( 1 / (sigma*sqrt(2*M_PI) ) ) * exp( -0.5 * pow( (meas-mean) / sigma,2));
}

int P = 50;  //Number of particles
Eigen::MatrixXf state_previous(3,1);
Eigen::MatrixXf state_current(3,1);

Eigen::MatrixXf meas(3,1);
Eigen::MatrixXf e(3,1);
Eigen::MatrixXf d(3,1);
Eigen::MatrixXf X(3,P);
Eigen::MatrixXf w(3,P);
Eigen::MatrixXf W(3,P);
std::default_random_engine generator;
Eigen::MatrixXf Xp(3,P);
float mean_x;
float mean_y;
float mean_yaw;
float dt = 0.05;
float Q = 0.1;
float R = 0.1;
float seed;
double ips_xp;
double ips_yp;
double ips_yawp;
quat q;
geometry_msgs::Pose pose; 
geometry_msgs::Pose pose_real;
geometry_msgs::PoseArray pose_array;
visualization_msgs::Marker marker;

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "real_positioning");
    ros::NodeHandle n_pos;

    //Subscribe to the desired topics and assign callbacks
    //ros::Subscriber pose_sub = n_pos.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber pose_sub = n_pos.subscribe("/indoor_pos",1,pose_callback);
    ros::Subscriber map_sub = n_pos.subscribe("/map", 1, map_callback);
    ros::Subscriber odom_sub = n_pos.subscribe("/odom",1,odom_callback);

    std::normal_distribution<double> distribution_d(0,Q);
    std::normal_distribution<double> distribution_e(0,R);
    std::uniform_real_distribution<double> distribution_i(-2,2);
    std::uniform_real_distribution<double> rand(0,1);

    //Setup topics to Publish from this node
    pose_real_publisher = n_pos.advertise<geometry_msgs::Pose>("/pose_real",1,true);
    pose_publisher = n_pos.advertise<geometry_msgs::Pose>("/pose",1,true);
    pose_array_publisher = n_pos.advertise<geometry_msgs::PoseArray>("/pose_array",1,true);
    marker_pub = n_pos.advertise<visualization_msgs::Marker>("/marker",1,true);
    pose_array.header.frame_id = "/base_link";
    marker.header.frame_id = "/base_link";
    marker.scale.x = 0.3;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;

    for(int i=0;i<P;i++){
        X(0,i) = distribution_i(generator);
        X(1,i) = distribution_i(generator);
        X(2,i) = distribution_i(generator);
    }

    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate
    state_previous(0) = -0.5;
    state_previous(1) = -0.5;
    state_previous(2) = -0.5;

    ips_x = 1.5;
    ips_y = 0.3;
    ips_yaw = 2*M_PI - M_PI/4;
    
    while (ros::ok()) {

	    loop_rate.sleep(); //Maintain the loop rate
	    ros::spinOnce();   //Check for new messages

        //ROS_INFO("odom_callback vx: %f vy: %f az: %f", odom_vx, odom_vy, odom_az);
        state_current(0) = state_previous(0) + odom_vx*cos(state_previous(2))*dt;
        state_current(1) = state_previous(1) + odom_vx*sin(state_previous(2))*dt;
        state_current(2) = state_previous(2) + odom_az*dt;
        //ROS_INFO("current states: x: %f, y: %f, z: %f", state_current(0),state_current(1),state_current(2));

        if(state_current(2)>2*M_PI){
        	state_current(2)=state_current(2)-2*M_PI;
        }
        
		//Measurement update

        meas(0) = ips_x;
	    meas(1) = ips_y;
	    meas(2) = ips_yaw;	

		//Particle filter

	    for(int i=0;i<P;i++){

	        Xp(0,i) = X(0,i) + odom_vx*cos(state_previous(2))*dt;
	        Xp(1,i) = X(1,i) + odom_vx*sin(state_previous(2))*dt;
	        Xp(2,i) = X(2,i) + odom_az*dt;
	        if(Xp(2,i)>2*M_PI){
        		Xp(2,i)=Xp(2,i)-2*M_PI;
        	}
        	if(Xp(2,i)<0){
        		Xp(2,i) = 2*M_PI + Xp(2,i);
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
	    	for(int j=0;j<P;j++){
	    		if(W(0,j)>W(0,P-1)*seed){
	    			X(0,i) = Xp(0,j);
					break;
	    		}
	    	}
	    	for(int j=0;j<P;j++){
                if(W(1,j)>W(1,P-1)*seed){
                	X(1,i) = Xp(1,j);
                	break;
                }
	    	}
	    	for(int j=0;j<P;j++){
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

			if(X(2,i)>2*M_PI){
				X(2,i) = X(2,i) - 2*M_PI;
			}	
			q = e2q(0,0,X(2,i));
			pose.orientation.x = q.x;
			pose.orientation.y = q.y;
			pose.orientation.z = q.z;
			pose.orientation.w = q.w;

		    pose_array.poses.push_back(pose);			    
		}

        pose_real.position.x = ips_x;
        pose_real.position.y = ips_y;
        q = e2q(0,0,ips_yaw);
        pose_real.orientation.x = q.x;
        pose_real.orientation.y = q.y;
        pose_real.orientation.z = q.z;
        pose_real.orientation.w = q.w;

        marker.pose = pose_real;

		mean_x = mean(X, 0);
		mean_y = mean(X, 1);
		mean_yaw = mean(X, 2);


        if(fabs(ips_xp-ips_x)>0.05){
            ROS_INFO("New x received");
            for(int i=0;i<P;i++){
                X(0,i) = ips_x + distribution_i(generator);
            }
        }
        if(fabs(ips_yp-ips_y)>0.05){
            ROS_INFO("New y received");
            for(int i=0;i<P;i++){
                X(1,i) = ips_y + distribution_i(generator);
            }
        }
        if(fabs(ips_yawp-ips_yaw)>0.05){
            ROS_INFO("New yaw received");
            for(int i=0;i<P;i++){
                X(2,i) = ips_yaw + distribution_i(generator);
            }
        }

        state_previous(0) = mean_x;
        state_previous(1) = mean_y;
        state_previous(2) = mean_yaw;

        pose.position.x = mean_x;
        pose.position.y = mean_y;

        q = e2q(0,0,mean_yaw);
        pose.orientation.x = q.x;
        pose.orientation.y = q.y;
        pose.orientation.z = q.z;
        pose.orientation.w = q.w;


		pose_array_publisher.publish(pose_array);
        pose_publisher.publish(pose);
        pose_real_publisher.publish(pose_real);
        marker_pub.publish(marker);

        //ROS_INFO("ips_x : %f, mean_x: %f",ips_x, mean_x);
        //ROS_INFO("ips_y: %f, mean_y: %f", ips_y, mean_y);

        ips_xp = ips_x;
        ips_yp = ips_y;
        ips_yawp = ips_yaw;


    }
    return 0;
}
