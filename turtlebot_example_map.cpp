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

double ips_x; //x position from IPS
double ips_y; //y position from IPS
double ips_yaw; //Yaw (aka angle relative to z-axis) from IPS

double angle_min; //Lower bound for field of vision of laser scan
double angle_max; //Upper bound for field of vision of laser scan
double angle_increment; //Angular distance between each consecutive laser beam
double time_increment; //Not important (i think)
double scan_time; //Not important
double range_min; //Minimum range that laser can travel (i don't use in this code, maybe i should)
double range_max; //Maximum range that laser can travel 
std::vector<float> ranges; //Vector of floats that will contain the distances each laser travelled
float res = 10; //Resolution of the occupancy grid. Note that I assume a base size of 10x10 - and then the resolution splits each grid into subgrids
               //So for example, if resolution is 5, each single grid is broken into a 5x5 subgrid, so the resulting map is then 50x50
int gridSize = 10*res; //Per the previous comment, this value determines the size of the occupancy grid


nav_msgs::OccupancyGrid beliefMap; //This is the map which will be updated and published
nav_msgs::MapMetaData map_metadata = beliefMap.info; //The map topic has a metadata field which contains things like how big it is 

short sgn(int x) { return x >= 0 ? 1 : -1; } //If x>=0, return 1, else return -1

void pose_callback(const gazebo_msgs::ModelStates &msg) { //Callback function for the Position topic (SIMULATION)
    int i;
    for (i = 0; i < msg.name.size(); i++)
        if (msg.name[i] == "mobile_base")
            break;

    ips_x = msg.pose[i].position.x + 5; // In the simulation, the point (0,0) is at the center of the grid. I find it easier if we map (0,0) to the bottom left of the grid, which
                                        // is the equivalent transformation of shifting the (-5,-5) to (0,0), which is what is done here
    ips_y = msg.pose[i].position.y + 5;
    ips_yaw = tf::getYaw(msg.pose[i].orientation); //The orientation field of the pose topic is a quaternion. The function tf::getYaw converts the quaternion into yaw
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
void map_callback(const nav_msgs::OccupancyGrid &msg) { //Not sure how we should use this
    //This function is called when a new map is received

    //you probably want to save the map into a form which is easy to work with
}

void scan_callback(const sensor_msgs::LaserScan &msg){ //This function is called when laser scan data is received

    angle_min = msg.angle_min; //See where these variables are declared above to know what they represent
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
void bresenham(int x0, int y0, int x1, int y1, std::vector<long int> &x, std::vector<long int> &y) { // Bresenham algorithm which was coded for us

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
    ros::init(argc, argv, "mapping");
    ros::NodeHandle n_map;
    tf::Transform transform; //This transform is used to map our occupancy grid to the base frame in rviz
    tf::TransformBroadcaster br;

    int angles; //Number of angles in each set of scan data
    int x0; //See bresenham
    int y0;
    int x1;
    int y1;
    float V; //Linear velocity of robot - note this is relative to the robot frame
    float U; //Angular velocity of robot
    double angle; //See below
    std::vector<long int> x; //The vectors x and y are acted on by the bresenham function. They are vectors which correspond to the cells in the map through which a laser travelled
    std::vector<long int> y;
    double size;
    map_metadata.resolution = 1/res; //All this metadata is a description of our map
    map_metadata.width = 10*res;
    map_metadata.height = 10*res;
    map_metadata.origin.position.x = 0;
    map_metadata.origin.position.y = 0;
    map_metadata.origin.position.z = 0;
    map_metadata.origin.orientation.x = 0;  
    map_metadata.origin.orientation.y = 0;
    map_metadata.origin.orientation.z = 0;
    beliefMap.info = map_metadata;

    Eigen::MatrixXf map(gridSize, gridSize); //Initialize map
    map.setZero(); //Set all probabilities in map to 0 initially

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n_map.subscribe("/gazebo/model_states", 1, pose_callback); //Subscribe to IPS from gazebo
    //ros::Subscriber map_sub = n_map.subscribe("/map", 1, map_callback); //Not used
    ros::Subscriber scan_sub = n_map.subscribe("/scan",1,scan_callback); //Subscribe to the laser scans

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n_map.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1); //Velocity publisher
    //pose_publisher = n_map.advertise<geometry_msgs::PoseStamped>("/pose", 1, true); //Not used here
    //marker_pub = n_map.advertise<visualization_msgs::Marker>("visualization_marker", 1, true); //Also not used
    ros::Publisher map_pub = n_map.advertise<nav_msgs::OccupancyGrid>("/beliefMap",1,true); //Map publisher

    //Velocity control variable
    geometry_msgs::Twist vel;
    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate


    while (ros::ok()) {


        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messagess

    	transform.setOrigin(tf::Vector3(5,5,0));
    	transform.setRotation(tf::Quaternion(0,0,0,1));
    	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"/map","base_link")); //These lines transform our map so it overlays the grid shown in rviz

    	if(!ranges.empty()){ //Initially, the ranges value is empty, so if that's the case do not enter main body of code
    	    
    	    angles = round((angle_max - angle_min)/angle_increment); //Find number of angles through which to loop
           	    //ROS_INFO("# Angles: %d", angles);
            for(int i=0;i<angles;i++){ //Loop through all the angles
        		if(!(std::isnan(ranges[i]))){
            	 	angle = (ips_yaw + angle_min) + i*angle_increment; //Pick an angle
            		x.clear(); //Clear x and y so we don't retain old values
             	   	y.clear();

        

        		    x0 = floor(ips_x*res);
        		    y0 = floor(ips_y*res);

                    x1 = floor((ips_x + ranges[i]*cos(angle))*res);
                    y1 = floor((ips_y + ranges[i]*sin(angle))*res); //These lines obtain the position of the robot and the position of the object which a laser hit 
                    

                    bresenham(x0,y0,x1,y1,x,y);
                    Eigen::ArrayXXf bresenhamResult(x.size(),3);
        		    for(int j=0;j<x.size();j++){//Map bresenham x and y vectors to bresenham results map                     

                        bresenhamResult(j, 0) = x[j];

                        bresenhamResult(j, 1) = y[j];

                        if(j == (x.size()-1)){
                            if(!(std::isnan(ranges[i]))){
                                bresenhamResult(j,2) = 0.7;  
                            } 
                            else if(std::isnan(ranges[i])){
                                bresenhamResult(j,2) = 0.3;
                            }
                        }
                        else if(j<(x.size()-1)){
                            bresenhamResult(j,2) = 0.3;
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
                        map(y[j], x[j]) = map(y[j], x[j]) + log(bresenhamResult(j,2)/(1-bresenhamResult(j,2)));
                    }
            			 	
                    
                    beliefMap.data.clear();
                        for(int p=0;p<map.cols();p++){
                            for(int l=0;l<map.rows();l++){
                                beliefMap.data.push_back( 100 * ( exp(map(p,l)) / ( 1+exp(map(p,l)) ) ) );
                            }
                        }    
                   
                    map_pub.publish(beliefMap); 
                
                }
		    }		
	    } 	
    }
    return 0;
}
