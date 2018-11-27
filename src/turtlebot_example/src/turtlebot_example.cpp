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
#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

visualization_msgs::Marker marker;
ros::Publisher marker_publisher;
#define TAGID 0

//Callback function for the Position topic (LIVE)

double XPos,YPos, theta;
Eigen::MatrixXd map(100,100);


void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  //This function is called when a new position message is received
  double XPos = msg.pose.pose.position.x; // Robot X psotition
  double YPos = msg.pose.pose.position.y; // Robot Y psotition
  double theta = tf::getYaw(msg.pose.pose.orientation); // Robot Yaw

  // std::cout << "X: " << XPos << ", Y: " << YPos << ", Yaw: " << theta << std::endl ;
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
   marker_publisher.publish(lines);

}



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

// std::vector<std::vector<int> > sortDistances(const Eigen::MatrixXf &distances, const Eigen::MatrixXd &nodeList){
//   ROS_INFO("Entered");
//   int index = -1;
//   int misplacedValue = -1;
//   int temp = -1;
//   float tempDistance;
//   float tempIndex;

//   for(int i = 0; i < distances.size(); i++){ //Copy Vector
//     dist[0] = distances[i];
//     dist[1] = i;
//     sortedDistances.push_back(dist);
//   }
//   for(int i = 1; i < sortedDistances.size(); i++){ //Insertion sort

//     if(sortedDistances[i][0] < sortedDistances[i-1][0]){
      
//       for(int j = i; j>=0; j--){
//         if(sortedDistances[j][0]<sortedDistances[j-1][0]){
//           tempDistance = sortedDistances[j+1][0];
//           tempIndex = sortedDistances[j+1][1];
//           sortedDistances[j][0] = sortedDistances[j-1][0];
//           sortedDistances[j][1] = sortedDistances[j-1][1];
//           sortedDistances[j-1][0] = tempDistance;
//           sortedDistances[j-1][1] = tempIndex;
//         }
//       }
//     }

//   }
//   return sortedDistances;

// }


double get_dist(double x1, double y1, double x2, double y2){
  return sqrt( pow(x1-x2,2) + pow(y1-y2,2) );
}

Eigen::MatrixXf sort(Eigen::ArrayXf &list){

  //ROS_INFO("in sorting function");
  int numElements = list.size();
 // ROS_INFO("size: %d",numElements);
  Eigen::MatrixXf sorted(2,numElements);

  for(int i=0;i<numElements;i++){
    sorted(0,i) = list(i);
    sorted(1,i) = i;
  }
  float temp;
  int temp_ind;
  float smallest;
  int smallest_ind = 0;
  int original_ind = 0;
  //Bad sorting algorithm
  for(int i=0;i<numElements;i++){
    smallest = sorted(0,i);
  //  ROS_INFO("smallest: %f",smallest);
    for(int j=i;j<numElements;j++){
     // ROS_INFO("j: %d",j);
      if(sorted(0,j)<=smallest){
        smallest = sorted(0,j);
       // ROS_INFO("smallest: %f",smallest);
        smallest_ind = j;
        original_ind = sorted(1,j);
       // ROS_INFO("smallest_ind: %d",smallest_ind);
      }
    }
    temp = sorted(0,i);
    temp_ind = sorted(1,i);
    sorted(0,i) = smallest;
    sorted(1,i) = original_ind;
    sorted(0,smallest_ind) = temp;
    sorted(1,smallest_ind) = temp_ind;


  }

  for(int i=0;i<numElements;i++){
    //ROS_INFO("%d: %f, %f: %f",i,sorted(0,i),sorted(1,i),list(sorted(1,i)));
    //ROS_INFO("original index of element: %f",sorted(1,i));
    if(sorted(0,i) != list(sorted(1,i))){
      //Something went wrong
      ROS_INFO("Sorting went wrong.");
    }
  }

  return sorted;
}

bool check_collision(double x1, double y1, double x2, double y2, const Eigen::MatrixXf &obstacleCoords, int numObstacles){
  //ROS_INFO("checking collision");
  double inc = 0.1;
  int steps = 10;
  double dx = x2 - x1;
  double dy = y2 - y1;
  double buffer = 0.3;
  double x;
  double y;
  double dist;
  for(int i=0;i<=steps;i++){
    x = x1 + i*inc*dx;
    y = y1 + i*inc*dy;
    for(int j=0;j<numObstacles;j++){

      dist = get_dist(x,y,obstacleCoords(0,j),obstacleCoords(1,j));
      //ROS_INFO("x1: %f, y1: %f, x2: %f, y2: %f, dist: %f",x,y,obstacleCoords(0,j),obstacleCoords(1,j),dist);
      if(dist<buffer){
        //ROS_INFO("collision found between: x1: %f, y1: %f, x2: %f, y2: %f",x,y,obstacleCoords(0,j),obstacleCoords(1,j));
        return true;
        
      }
    }

  }
  return false;
}

Eigen::MatrixXf GenerateProbabilisticRoadMap(Eigen::MatrixXd &obstacleMap, Eigen::MatrixXd &nodeMap, int numNodes){
 // ROS_INFO("Running");
  Eigen::MatrixXf points(3,numNodes);  
  int mapWidth = 100;
  int mapLength = 100;
  double dist;
  double buffer = 0.3;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,10.0);
  int obstacleFoundFlag = 0;
  int numObstacles = 0;
  int numMilestones = numNodes;
  visualization_msgs::Marker line_list;

  for(int i = 0; i < numNodes; i++){ //Generate nodes at random
    points(0,i) = distribution(generator);
    points(1,i) = distribution(generator);
    points(2,i) = 0;
  }


  //Find number of obstacles
  for(int i=0;i<mapWidth;i++){
    for(int j=0;j<mapLength;j++){
      if (obstacleMap(i,j) > 1){
        numObstacles += 1;
      }
    }
  }
  
  Eigen::MatrixXf obstacleCoords(2,numObstacles);
  int index = 0;
  for(int i=0;i<mapWidth;i++){
    for(int j=0;j<mapLength;j++){
      if (obstacleMap(i,j) > 1){
        obstacleCoords(0,index) = j*0.1;
        obstacleCoords(1,index) = i*0.1;
        index += 1;
      }
    }
  }
  
  //Check interfering nodes

  for(int i=0;i<numNodes;i++){
    for(int j=0;j<numObstacles;j++){
      dist = get_dist(points(0,i),points(1,i),obstacleCoords(0,j),obstacleCoords(1,j));
      if(dist <= buffer && points(2,i) == 0){
        // ROS_INFO("Interference found");
        points(2,i) = 1; //Change colour to white
        numMilestones -= 1;
      }
    }
  }

  //Create milestones list
  Eigen::MatrixXf milestones(2,numMilestones);
  index = 0;
  for(int i=0;i<numNodes;i++){
    if(points(2,i) == 0){
      milestones(0,index) = points(0,i);
      milestones(1,index) = points(1,i);
      index += 1;
    }
  }
  
  Eigen::ArrayXf dists(numMilestones);
  Eigen::MatrixXf sorted_dists(2,numMilestones);
  Eigen::MatrixXd connections = Eigen::MatrixXd::Zero(numMilestones,numMilestones);
  int n = 10; //Number of closest points to look at

  //Connect milestones
  for(int i=0;i<numMilestones;i++){
    for(int j=0;j<numMilestones;j++){

      dists(j) = get_dist(milestones(0,i),milestones(1,i),milestones(0,j),milestones(1,j));
    }

    sorted_dists = sort(dists);

    for(int k=0;k<n;k++){
      //Keep in mind: i represents the current milestone
      //ROS_INFO("index: %f, x: %f, y: %f",sorted_dists(1,k),milestones(0,sorted_dists(1,k)),milestones(1,sorted_dists(1,k)));
      if (!check_collision(milestones(0,i),milestones(1,i),milestones(0,sorted_dists(1,k)),milestones(1,sorted_dists(1,k)),obstacleCoords,numObstacles)){
        //ROS_INFO("No collision");
        connections(i,sorted_dists(1,k)) = 1;
        connections(sorted_dists(1,k),i) = 1; //If i is connected to j, then j is connected to i
      }
    }
  }

  line_list.id = 1;
  line_list.header.frame_id = "/base_link";
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1;
  line_list.scale.y = 0.1;
  line_list.scale.z = 0.1;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  geometry_msgs::Point p;
  line_list.points.clear();
  for(int i=0;i<numMilestones;i++){
    for(int j=0;j<numMilestones;j++){
      if(connections(i,j) == 1){
        p.x = milestones(0,i);
        p.y = milestones(1,i);
        line_list.points.push_back(p);
        p.x = milestones(0,j);
        p.y = milestones(1,j);
        line_list.points.push_back(p);

      }
    }
  }

  marker_publisher.publish(line_list);


  return points;


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
    marker_publisher = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    //Velocity control variable
    geometry_msgs::Twist vel;
    geometry_msgs::Point point;
    Eigen::MatrixXf points;
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    int numNodes = 300;
    std_msgs::ColorRGBA color;

    while (ros::ok())
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages


      Eigen::MatrixXd nodeMap = Eigen::MatrixXd::Zero(100,100);      
      points = GenerateProbabilisticRoadMap(map,nodeMap,numNodes);
      marker.id = 0;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.header.frame_id = "/base_link";
      marker.type = visualization_msgs::Marker::POINTS;
      marker.points.clear();
      marker.colors.clear();
      for(int i = 0; i < numNodes; i++){
        point.x = points(0,i);
        point.y = points(1,i);
        color.r = points(2,i);
        color.g = points(2,i);
        color.b = points(2,i);
        color.a = 1;
        marker.points.push_back(point);
        marker.colors.push_back(color);
      }

      //Main loop code goes here:
      vel.linear.x = 0; // set linear speed
      vel.angular.z = 0; // set angular speed
      
      velocity_publisher.publish(vel); // Publish the command velocity
      marker_publisher.publish(marker);

    }
    return 0;
}