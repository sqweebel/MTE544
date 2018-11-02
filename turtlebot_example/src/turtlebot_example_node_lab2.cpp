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
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
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

int main(int argc, char **argv) {
    //Initialize the ROS framework
    ros::init(argc, argv, "main_control");
    ros::NodeHandle n;
    std::ofstream logFile;
    logFile.open("/home/colin/lab2LogFile.txt");

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber scan_sub = n.subscribe("/scan",1,scan_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>("/pose", 1, true);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20); //20Hz update rate

    while (ros::ok()) {
        //Main loop code goes here:
        vel.linear.x = 0;  // set linear speed
        vel.angular.z = 0; // set angular speed

        velocity_publisher.publish(vel); // Publish the command velocity
        loop_rate.sleep(); //Maintain the loop rate
        ros::spinOnce();   //Check for new messagess

	/*for(int k=0;k<100000000;k++){
		wait
	}*/

	if(!ranges.empty() && angle_min && angle_increment){
	    int angles;
	    int x0;
	    int y0;
   	    int x1;
 	    int y1;
	    double angle;
	    std::vector<int> x;
	    std::vector<int> y;
	    double size;
	    angles = round((angle_max - angle_min)/angle_increment);
       	    /*ROS_INFO("# Angles: %d", angles);*/
            for(int i=0;i<angles;i++){
		
	 	angle = (ips_yaw + angle_min) + i*angle_increment;
		x.clear();
 	   	y.clear();
		/*ROS_INFO("angle_min: %f",angle_min);
		ROS_INFO("Yaw: %f",ips_yaw);
		ROS_INFO("angle_increment: %f",angle_increment);
		ROS_INFO("angle %f",angle); */
   		
		if(!(std::isnan(ranges[i]))){
		    x0 = round(ips_x);
		    y0 = round(ips_y);
		    x1 = round(ips_x + ranges[i]*cos(angle));
		    y1 = round(ips_y + ranges[i]*sin(angle));

		    logFile << ips_yaw << "\n";
		    logFile << angle << "\n";
		    logFile << x0 << ", " << y0 << "\n";
	            logFile << x1 << ", " << y1 << "\n";
		    logFile << x.size() << "\n";

		    ROS_INFO("Angle: %f", angle);
		    ROS_INFO("x0: %d, y0: %d", x0, y0);
		    ROS_INFO("x1: %d, y1: %d", x1, y1);

		    bresenham(x0,y0,x1,y1,x,y);
		    size = x.size();
		    ROS_INFO("Vector size: %f",size);


		}
		
	    }
	    logFile.close();
	    break;
	
	}
	
    }
    return 0;
}
