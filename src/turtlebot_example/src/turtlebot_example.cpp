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

void a_star(Eigen::MatrixXf &milestones, Eigen::MatrixXd &connections, int start, int end, int numMilestones){
  ROS_INFO("running a_star");
  //Find edge lengths
  Eigen::MatrixXf dists = Eigen::MatrixXf::Zero(numMilestones,numMilestones);
  int o_size = 1; //size of open set
  int c_size = 0; //size of closed set
  Eigen::MatrixXf open(4,o_size);
  Eigen::MatrixXf open_temp(4,o_size);
  Eigen::MatrixXf closed;
  double dmax;
  visualization_msgs::Marker line_list2;
  ROS_INFO("start: %d, starting node x: %f y: %f",start,milestones(0,0),milestones(1,0));
  ROS_INFO("end: %d, ending node x: %f y: %f",end,milestones(0,1),milestones(1,1));
  for(int i=0;i<numMilestones;i++){
    for(int j=0;j<numMilestones;j++){
      if(connections(i,j)){
        dists(i,j) = get_dist(milestones(0,i),milestones(1,i),milestones(0,j),milestones(1,j));
        dists(j,i) = dists(i,j);
      }
    }
  }

  dmax = get_dist(milestones(0,start),milestones(1,start),milestones(0,end),milestones(1,end));
  ROS_INFO("dmax: %f",dmax);

  open(0,0) = start; //node position
  open(1,0) = 0; // back pointer
  open(2,0) = dmax; //lower bound cost
  open(3,0) = 0; // current cost
  ROS_INFO("open set made");

  bool done = 0;
  float best;
  int best_ind;
  while(!done){
    //find best node in open set
    best = 1000;
    for(int i=0;i<o_size;i++){
      if(open(2,i)<=best){
        best = open(2,i);
        best_ind = i;
        //ROS_INFO("found new best");
      }
    }
    //ROS_INFO("best node is at: %d, corresponds to node: %f",best_ind,open(0,best_ind));
    //move best node to closed set
    c_size += 1;
    //ROS_INFO("c_size: %d",c_size);
    closed.conservativeResize(4,c_size);

    closed(0,c_size-1) = open(0,best_ind);
    closed(1,c_size-1) = open(1,best_ind);
    closed(2,c_size-1) = open(2,best_ind);
    closed(3,c_size-1) = open(3,best_ind);
    ROS_INFO("added: %f, %f, %f, %f",closed(0,c_size-1),closed(1,c_size-1),closed(2,c_size-1),closed(3,c_size-1));
    //ROS_INFO("added best node to closed set");
    //Check if end reached
    if(open(0,best_ind)==end){
      //ROS_INFO("current ind at stop: %f",open(0,best_ind));
      done = 1;
      continue;
    }
    int numNeighbours = 0;
    //Get all neighbours of best node
   // ROS_INFO("current ind: %f",open(0,best_ind));
    for(int i=0;i<numMilestones;i++){
      if(connections(open(0,best_ind),i)){
        //ROS_INFO("connect between %f and %d",open(0,best_ind),i);
        numNeighbours += 1;
      }
    }
    //ROS_INFO("node has %d neighbours",numNeighbours);
    Eigen::ArrayXf neighbours(numNeighbours);
    int index = 0;
    for(int i=0;i<numMilestones;i++){
      //ROS_INFO("i is: %d",i);
      if(connections(open(0,best_ind),i)){

        neighbours(index) = i;
        index += 1;
        //ROS_INFO("neighbour at: %d",i);
      }
    }
    //ROS_INFO("finished finding neighbours");
    bool found_closed;
    bool found_open;
    double dtogo;
    double dcur;
    int found_open_ind;
    for(int i=0;i<numNeighbours;i++){
      //Check if neighbour is already in closed set
      found_closed = false;
      for(int j=0;j<c_size;j++){
        if(neighbours(i) == closed(0,j)){
          found_closed = true;
        }
      }
      if(found_closed){
        continue;
      }
      //ROS_INFO("checked if neighbour in closed set");

      dtogo = get_dist(milestones(0,neighbours(i)),milestones(1,neighbours(i)),milestones(0,end),milestones(1,end));
      dcur = open(3,best_ind) + dists(open(0,best_ind),neighbours(i));
      //ROS_INFO("distances calculated");
      for(int j=0;j<o_size;j++){
        found_open = false;
        if(neighbours(i) == open(0,j)){
          found_open = true;
          found_open_ind = j;
        }
      }
      //ROS_INFO("checked if neighbour in open set");
      if(!found_open){
        //Add this node to open set
        o_size += 1;
        //ROS_INFO("open size: %d",o_size);
        open.conservativeResize(4, o_size);
        open(0,o_size-1) = neighbours(i);
        open(1,o_size-1) = open(0,best_ind);
        open(2,o_size-1) = dtogo+dcur;
        open(3,o_size-1) = dcur;

        //ROS_INFO("added: %f, %f, %f, %f",open(0,o_size-1),open(1,o_size-1),open(2,o_size-1),open(3,o_size-1));

      }
      else{
        //ROS_INFO("found in open set at: %d",found_open_ind);
        //Node is already in open set. Check if its better than the node currently in its place.
        //ROS_INFO("neighbours(%d): %f",i,neighbours(i));
        //ROS_INFO("open set size: %ld",open.size());
        if(dcur < open(3,found_open_ind)){
          ROS_INFO("replacing");
          open(0,found_open_ind) = neighbours(i);
          open(1,found_open_ind) = open(0,best_ind);
          open(2,found_open_ind) = dtogo+dcur;
          open(3,found_open_ind) = dcur;
        }
        //SROS_INFO("past if statement");
      }
    }

    //ROS_INFO("open set modification finished");
    o_size -= 1;
    ROS_INFO("o_size: %d",o_size);
    ROS_INFO("c_size: %d",c_size);
    //ROS_INFO("new o size: %d",o_size);
    index = 0;
    open_temp.conservativeResize(4,o_size);
    for(int i=0;i<o_size+1;i++){
      if(i != best_ind){
        //ROS_INFO("index: %d",index);
        //ROS_INFO("open(0,i): %f, best_ind: %d",open(0,i),best_ind);
        open_temp(0,index) = open(0,i);
        open_temp(1,index) = open(1,i);
        open_temp(2,index) = open(2,i);
        open_temp(3,index) = open(3,i);
        index += 1;
        //ROS_INFO("open: %f, %f, %f, %f",open(0,i),open(1,i),open(2,i),open(3,i));
        //ROS_INFO("open_temp: %f, %f, %f, %f",open_temp(0,index-1),open_temp(1,index-1),open_temp(2,index-1),open_temp(3,index-1));
         
      }
    }
    open.conservativeResize(4,o_size);
    //ROS_INFO("changing open set");
    open = open_temp;
  }


  for(int i=0;i<c_size;i++){
    //ROS_INFO("closed(0,%d): %f",i,closed(0,i));
    //ROS_INFO("closed(1,%d): %f",i,closed(1,i));
    //ROS_INFO("closed(2,%d): %f",i,closed(2,i));
    //ROS_INFO("closed(3,%d): %f",i,closed(3,i));

  }



  line_list2.id = 2;
  line_list2.header.frame_id = "/base_link";
  line_list2.type = visualization_msgs::Marker::LINE_LIST;
  line_list2.scale.x = 0.1;
  line_list2.scale.y = 0.1;
  line_list2.scale.z = 0.1;
  line_list2.color.g = 1.0;
  line_list2.color.a = 1.0;

  geometry_msgs::Point p;
  line_list2.points.clear();
  for(int i=0;i<c_size;i++){
    for(int j=0;j<c_size;j++){
        p.x = milestones(0,closed(1,i));
        p.y = milestones(1,closed(1,i));
        line_list2.points.push_back(p);
        p.x = milestones(0,closed(1,j));
        p.y = milestones(1,closed(1,j));
        line_list2.points.push_back(p);
    }
  }

  marker_publisher.publish(line_list2);


ROS_INFO("finished planning");

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

  points(0,0) = 1;
  points(1,0) = 1;
  points(2,0) = 0;
  points(0,1) = 9;
  points(1,1) = 9;
  points(2,1) = 0;

  for(int i = 2; i < numNodes; i++){ //Generate nodes at random
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
  Eigen::MatrixXf milestones(2,numMilestones); //+2 because need to add start and end points
  milestones(0,0) = 1;
  milestones(1,0) = 1; //Start
  milestones(0,1) = 9;
  milestones(1,1) = 9; //end
  index = 2;
  for(int i=0;i<numNodes-2;i++){
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
      if (!check_collision(milestones(0,i),milestones(1,i),milestones(0,sorted_dists(1,k)),milestones(1,sorted_dists(1,k)),obstacleCoords,numObstacles) && (i != sorted_dists(1,k))){
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

  //The milestones and the connections have been made. Time to find the shortest path.

  std::vector<int> indices; //Indices of milestones which reprsesent shortest path to destination
  marker_publisher.publish(line_list);
  a_star(milestones, connections, 0, 1, numMilestones);
  

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