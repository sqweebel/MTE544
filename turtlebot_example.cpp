//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

ros::Publisher marker_pub;

#define TAGID 0

//Callback function for the Position topic (LIVE)

double XPos,YPos, theta;
Eigen::MatrixXd map(100,100);


void pose_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
	//This function is called when a new position message is received
	double XPos = msg.pose.pose.position.x; // Robot X psotition
	double YPos = msg.pose.pose.position.y; // Robot Y psotition
 	double theta = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

	std::cout << "X: " << XPos << ", Y: " << YPos << ", Yaw: " << theta << std::endl ;
}

//Example of drawing a curve
void drawCurve(int k) 
{
   // Curves are drawn as a series of stright lines
   // Simply sample your curves into a series of points

   double x = 0;
   double y = 0;
   double steps = 50;

   visualization_msgs::Marker lines;
   lines.header.frame_id = "/map";
   lines.id = k; //each curve must have a unique id or you will overwrite an old ones
   lines.type = visualization_msgs::Marker::LINE_STRIP;
   lines.action = visualization_msgs::Marker::ADD;
   lines.ns = "curves";
   lines.scale.x = 0.1;
   lines.color.r = 1.0;
   lines.color.b = 0.2*k;
   lines.color.a = 1.0;

   //generate curve points
   for(int i = 0; i < steps; i++) {
       geometry_msgs::Point p;
       p.x = x;
       p.y = y;
       p.z = 0; //not used
       lines.points.push_back(p); 
       //curve model
       x = x+0.1;
       y = sin(0.1*i*k);   
   }

   //publish new curve
   marker_pub.publish(lines);

}

/*
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
*/

//Callback function for the map
void map_callback(const nav_msgs::OccupancyGrid& msg)
{
    //This function is called when a new map is received
    
    //you probably want to save the map into a form which is easy to work with
    for(int i = 0; i < 100; i++){
      for(int j = 0; j < 100; j++){
          map(i,j) = msg.data[i*100 + j];
      }      
    }
    

}

std::vector<int> sortDistances(const std::vector<int> distance){

  int index = -1;
  int misplacedValue = -1;
  int temp = -1;
  std::vector<std::vector<int> > sortedDistances;
  for(int i = 0; i < distance.size(); i++){ //Copy Vector
    sortedDistances[i][0] = distance[i];
    sortedDistances[i][1] = i; //Copy Index
  }
  for(int i = 0; i < sortedDistances.size(); i++){ //Insertion sort

    if(sortedDistances[i+1][0] < sortedDistances[i][0]){
      
      for(int j = i; j>=0; j--){
        if(sortedDistances[j][0]<sortedDistances[j+1][0]){
          tempDistance = sortedDistances[j+1][0];
          tempIndex = sortedDistances[j+1][1];
          sortedDistances[j][0] = sortedDistances[j+1][0];
          sortedDistances[j][1] = sortedDistances[j+1][1];
          sortedDistances[j+1][[0]= tempDistance;
          sortedDistances[j+1][[1]= tempIndex;
        }
      }
    }


  }
  return sortedDistances;

}

void GenerateProbabilisticRoadMap(Eigen::MatrixXd &obstacleMap, Eigen::MatrixXd &nodeMap){
  int row;
  int column;
  int numNodes = 100;
  int mapWidth = 100;
  int mapLength = 100;
  int obstacleFoundFlag = 0;
  std::vector<std::vector<int> > particleLocations;
  <std::vector<int> distance;
  for(int i = 0; i < numNodes; i++){ //Place particles

    row = rand()%mapWidth;
    column = rand()%mapLength;
    while(obstacleMap(row,column) != 0){
      row = rand()%mapWidth;
      column = rand()%mapLength;
    }
    ROS_INFO("column: %i \n", column);
    ROS_INFO("row   : %i \n", row);
    nodeMap(row,column) = 1;
    particleLocations[i][0] = row;
    particleLocations[i][1] = column;
  }
//
  for(int i = 0; i < numNodes; i++){ //eliminate invalid particles
    for (int j = 0; j < numNodes; j++){
      if (nodeMap(i,j) == 1 && obstacleMap(i,j) != 0){ //if there's both a particle and an obstacle
        nodeMap(i,j) = 0;
      }
      if (nodeMap(i,j) == 1 && obstacleMap(i,j) == 0){ //if there's a particle and no obstable
        for (int n = i-2; n <= i+2; n++){
          for (int m = j-2; m <= j+2; m++){
            if (obstacleMap(n,m) != 0){
              nodeMap(i,j) = 0;
              obstacleFoundFlag = 1;
              break;
            }
          }
          if (obstacleFoundFlag ==1){
            break;
          }
        }
      }
    }
  }

  for(int i = 0; i < particleLocations.size(); i++){ //Normalize
    for(int j = 0; j < particleLocations.size();j++){
      distance.push_back(sqrt((particleLocations[i,0] - particleLocations[j,0])^2 + (particleLocations[i,1] - particleLocations[j,1])^2));
    }

  }
  
  std::vector<std::vector<int> > sortedDistances = sortDistances(distance);


  int numConnections = 10;
  for(int i = 0; i<numConnections;i++){

  }


}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
    ros::Subscriber map_sub = n.subscribe("/map", 1, map_callback);
    ros::Subscriber pose_sub = n.subscribe("/gazebo/model_states", 1, pose_callback);

    //Setup topics to Publish from this node
    ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    
    //Velocity control variable
    geometry_msgs::Twist vel;

    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate

    std::ofstream file;
    file.open("/home/vincent/lab3Map.txt"); 
    Eigen::MatrixXd nodeMap(100,100);
	

    while (ros::ok())
    {
    	loop_rate.sleep(); //Maintain the loop rate
    	ros::spinOnce();   //Check for new messages

	 //Draw Curves
         drawCurve(1);
         drawCurve(2);
         drawCurve(4);
            
      GenerateProbabilisticRoadMap(map, nodeMap);
    	
      for(int i = 0; i < 100; i++){
        for(int j = 0; j < 100; j++){
          file << nodeMap(i,j) << ", ";
        }
        file << "\n";
      }
      file << "\n\n\n\n\n ---------------------------------------- \n\n\n\n\n\n\n\n";

      //Main loop code goes here:
    	vel.linear.x = 0.1; // set linear speed
    	vel.angular.z = 0.3; // set angular speed
      
    	velocity_publisher.publish(vel); // Publish the command velocity
    }

    return 0;
}
