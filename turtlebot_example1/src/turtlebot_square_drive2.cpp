//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various 
// inputs and outputs.
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <string>
#include <math.h>

double X = 0;
double Y = 0;
double Yaw = 0;

//Callback function for the Position topic 
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X psotition
//	double Y = msg->pose.pose.position.y; // Robot Y psotition
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	X = msg->pose.pose.position.x; // Robot X psotition
	Y = msg->pose.pose.position.y; // Robot Y psotition
 	Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw
	if (Yaw < 0){
		Yaw = 2*3.141592654 + Yaw;
	}

}



int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

	double X_i = X;
	double Y_i = Y;
	double Yaw_i = Yaw;


	double X_c = 0;
	double Y_c = 0;
	double Yaw_c = 0;

	bool linear = 1;
	bool rotating = 0;
	bool roll_over = 0;
	
	double distance = 0;
	double pi = 3.141592654;
	double Yaw_target = 0;

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages


 
    	//Main loop code goes here:

	
		
	

	if (linear){	
		vel.linear.x = 0.2;

		distance = sqrt(pow(X - X_i, 2) + pow(Y - Y_i, 2));
	
		if (distance > 1){
			vel.linear.x = 0;
			linear = 0;
			rotating = 1;
			Yaw_i = Yaw;
			distance = 0;
		}
	}
		
	if (rotating){
		vel.angular.z = 0.2;

		Yaw_target = Yaw_i + pi/2;

		if (Yaw_target > 2*pi) {
			roll_over = 1;
			Yaw_target = Yaw_target - 2*pi;
		}


		if (roll_over){
				if ( (Yaw < pi) && (Yaw > Yaw_target)){
				vel.angular.z = 0;
				rotating = 0;
				linear = 1;
				X_i = X;
				Y_i = Y;	
				}
		}

		if (!roll_over){
				if (Yaw > Yaw_target){
				vel.angular.z = 0;
				rotating = 0;
				linear = 1;
				X_i = X;
				Y_i = Y;	
				}
		}

		
	}

	ROS_INFO("Distance: %f/n", distance);
	ROS_INFO("Yaw: %f/n", Yaw);
	ROS_INFO("Yaw_i: %f/n", Yaw_i + 3.141592654/2);
	ROS_INFO("Yaw_target: %f/n", Yaw_target);
    	velocity_publisher.publish(vel); // Publish the command velocity




	


    }

    return 0;
}
