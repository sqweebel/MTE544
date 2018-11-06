//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various
// inputs and outputs needed for this lab
//
// Author: James Servos
// Edited: Nima Mohajerin
//
// //////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher pose_publisher;
ros::Publisher marker_pub;

double ips_x;
double ips_y;
double ips_yaw;

double angle_min;
double angle_max;
double angle_increment;
double time_increment;
double scan_time;
double range_min;
double range_max;
std::vector<float> ranges;
float res = 10;
int gridSize = 10*res;


nav_msgs::OccupancyGrid beliefMap;
nav_msgs::MapMetaData map_metadata = beliefMap.info;




short sgn(int x) { return x >= 0 ? 1 : -1; }
//Callback function for the Position topic (SIMULATION)
void pose_callback(const gazebo_msgs::ModelStates &msg) {

    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x + 5; //Add 5 to change coordinate frame
    ips_y = msg.pose[i].position.y + 5;
    ips_yaw = tf::getYaw(msg.pose[i].orientation);
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

void scan_callback(const sensor_msgs::LaserScan &msg){

    angle_min = msg.angle_min;
    angle_max = msg.angle_max;
    angle_increment = msg.angle_increment;
    scan_time = msg.scan_time;
    range_min = msg.range_min;
    range_max = msg.range_max;
    ranges = msg.ranges;


}

//Bresenham line algorithm (pass empty vectors)
// Usage: (x0, y0) is the first point and (x1, y1) is the second point. The calculated
//        points (x, y) are stored in the x and y vector. x and y should be empty
//	  vectors of integers and shold be defined where this function is called from.
void bresenham(int x0, int y0, int x1, int y1, std::vector<long int> &x, std::vector<long int> &y) {

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



int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;
    tf::Transform transform;
    tf::TransformBroadcaster br;
    std::ofstream logFile1;
    std::ofstream logFile2;
    logFile1.open("/home/colin/angles.txt");
    logFile2.open("/home/colin/map.txt");
    
    Eigen::MatrixXf map(gridSize, gridSize);
    map.setZero();
   
    //Eigen::MatrixXd map0 = map;
    //Subscribe to the desired topics and assign callbacks
    
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber scan_sub = n.subscribe("/scan",1,scan_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    ros::Publisher map_pub = n.advertise<nav_msgs::OccupancyGrid>("/beliefMap",1,true);
    ros::Publisher meta_map_pub = n.advertise<nav_msgs::MapMetaData>("/beliefMap_metadata",1,true);

    //Velocity control variable
    geometry_msgs::Twist vel;
    //Set the loop ratehttps://en.wikipedia.org/wiki/Row-_and_column-major_order
    ros::Rate loop_rate(20); //20Hz update rate

    //std::vector<std::vector<int>> mapArray(row, std::vector<int>(col));
    int count = 0;
    float V;
    float U;
    while (ros::ok()) {


        //Main loop code goes here:
        //vel.linear.x = 0;  // set linear speed
        //vel.angular.z = 0; // set angular speed
        //velocity_publisher.publish(vel); // Publish the command velocity

        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messagess

   //Main loop code goes here:
            

	transform.setOrigin(tf::Vector3(5,5,0));
	transform.setRotation(tf::Quaternion(0,0,0,1));
	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/map","base_link"));

	if(!ranges.empty()){
	    int angles;
	    int x0;
	    int y0;
   	    int x1;
 	    int y1;
	    double angle;
	    std::vector<long int> x;
	    std::vector<long int> y;
	    double size;
	    angles = round((angle_max - angle_min)/angle_increment);
       	    //ROS_INFO("# Angles: %d", angles);
            for(int i=0;i<angles;i++){
		
	 	angle = (ips_yaw + angle_min) + i*angle_increment;
		x.clear();
 	   	y.clear();

   		
		if(!(std::isnan(ranges[i]))){
		    logFile1 << angle << ": " << ranges[i] << "\n";
		    x0 = floor(ips_x*res);
		    y0 = floor(ips_y*res);
		    x1 = floor((ips_x + ranges[i]*cos(angle))*res);
		    y1 = floor((ips_y + ranges[i]*sin(angle))*res);
		    logFile1 << x0 << ", " << y0 << ", " << x1 << ", " << y1 << "\n";

		    bresenham(x0,y0,x1,y1,x,y);
                    Eigen::ArrayXXf bresenhamResult(x.size(),3);
		    
		    for(int j = 0; j<x.size();j++){//Map bresenham x and y vectors to bresenham results map                     

                        bresenhamResult(j, 0) = x[j];

                        bresenhamResult(j, 1) = y[j];

                        if(j == x.size()-1){
                            bresenhamResult(j,2) = 99;
                        }
                        else{
                            bresenhamResult(j,2) = 1;
                        }
			    if(y[j]>(10*res - 1)){
				y[j] = 10*res - 1;
			    }
			    if(x[j]>(10*res - 1)){
				x[j] = 10*res - 1;
			    }
			    if(y[j]<0){
			    	y[j] = 0;
			    }
			    if(x[j]<0){
				x[j] = 0;
			    }
			/*
			    if(map(19-y[j],x[j]) == 100){
				//do not alter
			    }
			    else{
                            map(19-y[j], x[j]) = map(19-y[j], x[j]) + log(bresenhamResult(j,2)/(100-bresenhamResult(j,2)));
			    }
			    if(map(19-y[j],x[j])>=1){
			    	map(19-y[j], x[j]) = 100;
			    }
			    if(map(19-y[j],x[j])<0){
				map(19-y[j],x[j]) = 0;
			    }    
			    
			*/

			    else{
                            map(y[j], x[j]) = map(y[j], x[j]) + log(bresenhamResult(j,2)/(100-bresenhamResult(j,2)));
			    }
			    if(map(y[j],x[j])>=100){
			    	map(y[j], x[j]) = 100;
			    }
			    if(map(y[j],x[j])<0){
				map(y[j],x[j]) = 0;
			    }   

			    beliefMap.data.clear();
			    for(int p=0;p<map.cols();p++){
			        for(int l=0;l<map.rows();l++){
				    beliefMap.data.push_back(map(p, l));
				}
 			    }    
			    map_metadata.resolution = 1/res;
			    map_metadata.width = 10*res;
			    map_metadata.height = 10*res;
     			    map_metadata.origin.position.x = 0;
     			    map_metadata.origin.position.y = 0;
     			    map_metadata.origin.position.z = 0;
     			    map_metadata.origin.orientation.x = 0;	
     			    map_metadata.origin.orientation.y = 0;
     			    map_metadata.origin.orientation.z = 0;
			    beliefMap.info = map_metadata;
			    map_pub.publish(beliefMap); 	
                    } 
		}		
		
	    } 
	//break;	
	}	
    }
	map(9,9) = 9;

	for(int j = 0; j < map.rows(); j++){
	    for(int k = 0; k < map.rows(); k++){
		logFile2 << map(j, k) << " ";
	     }
	       logFile2 << "\n";
	}
	for(int i=0;i<400;i++){
	    logFile2 << std::to_string(beliefMap.data[i]) << " ";
	}
	logFile1.close();
	logFile2.close();
    return 0;
}
