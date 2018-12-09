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
ros::Publisher marker_publisher;
#define TAGID 0
#define PI 3.141592654
//Callback function for the Position topic (LIVE)
double XPos,YPos, theta;
Eigen::MatrixXd map(100,100);
bool mapComplete = false;
void pose_callback(const gazebo_msgs::ModelStates &msg) {
  int i;
  for (i = 0; i < msg.name.size(); i++)
      if (msg.name[i] == "mobile_base")
          break;
  XPos = msg.pose[i].position.x;
  YPos = msg.pose[i].position.y;
  theta = tf::getYaw(msg.pose[i].orientation);
  if(theta<0){ //need to map
    theta += 2*M_PI;
  }
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
          if(map(i,j)>1){
            mapComplete = true;
          }
      }      
    }
}
double get_dist(double x1, double y1, double x2, double y2){
  return sqrt( pow(x1-x2,2) + pow(y1-y2,2) );
}
short sgn(int x) { return x >= 0 ? 1 : -1; }
std::vector<long int> x;
std::vector<long int> y;
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
bool check_collision(double x1, double y1, double x2, double y2, const Eigen::MatrixXf &obstacleCoords, int numObstacles, double buffer){
  //ROS_INFO("checking collision");
  double inc = 0.1;
  int steps = 10;
  double dx = x2 - x1;
  double dy = y2 - y1;
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
Eigen::ArrayXd a_star(Eigen::MatrixXf &milestones, Eigen::MatrixXd &connections, int start, int end, int numMilestones){
  //Find edge lengths
  Eigen::MatrixXf dists = Eigen::MatrixXf::Zero(numMilestones,numMilestones);
  int o_size = 1; //size of open set
  int c_size = 0; //size of closed set
  Eigen::MatrixXf open(4,o_size);
  Eigen::MatrixXf open_temp(4,o_size);
  Eigen::MatrixXf closed;
  double dmax;
  visualization_msgs::Marker closed_points;
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
      }
    }
    //move best node to closed set
    c_size += 1;
    closed.conservativeResize(4,c_size);
    closed(0,c_size-1) = open(0,best_ind);
    closed(1,c_size-1) = open(1,best_ind);
    closed(2,c_size-1) = open(2,best_ind);
    closed(3,c_size-1) = open(3,best_ind);
    //Check if end reached
    if(open(0,best_ind)==end){
      done = 1;
      continue;
    }
    int numNeighbours = 0;
    //Get all neighbours of best node
  
    for(int i=0;i<numMilestones;i++){
      if(connections(open(0,best_ind),i)){
        numNeighbours += 1;
      }
    }
    Eigen::ArrayXd neighbours(numNeighbours);
    int index = 0;
    for(int i=0;i<numMilestones;i++){
      if(connections(open(0,best_ind),i)){
        neighbours(index) = i;
        index += 1;
      }
    }
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
      dtogo = get_dist(milestones(0,neighbours(i)),milestones(1,neighbours(i)),milestones(0,end),milestones(1,end));
      dcur = open(3,best_ind) + dists(open(0,best_ind),neighbours(i));
      found_open = false;
      for(int j=0;j<o_size;j++){
        if(neighbours(i) == open(0,j)){
          found_open = true;
          found_open_ind = j;
        }
      }
      if(!found_open){
        //Add this node to open set
        o_size += 1;
        open.conservativeResize(4, o_size);
        open(0,o_size-1) = neighbours(i);
        open(1,o_size-1) = open(0,best_ind);
        open(2,o_size-1) = dtogo+dcur;
        open(3,o_size-1) = dcur;
      } else {
        //Node is already in open set. Check if its better than the node currently in its place.
        //ROS_INFO("Node %f is already in open set",open(0,found_open_ind));
        if(dcur < open(3,found_open_ind)){
          open(0,found_open_ind) = neighbours(i);
          open(1,found_open_ind) = open(0,best_ind);
          open(2,found_open_ind) = dtogo+dcur;
          open(3,found_open_ind) = dcur;
        }
      }
    }
    o_size -= 1;
    index = 0;
    open_temp.conservativeResize(4,o_size);
    for(int i=0;i<o_size+1;i++){
      if(i != best_ind){
        open_temp(0,index) = open(0,i);
        open_temp(1,index) = open(1,i);
        open_temp(2,index) = open(2,i);
        open_temp(3,index) = open(3,i);
        index += 1;
      }
    }
    open.resize(4,o_size);
    open = open_temp;
    closed_points.id = 2;
    closed_points.scale.x = 0.1;
    closed_points.scale.y = 0.1;
    closed_points.scale.z = 0.1;
    closed_points.header.frame_id = "/base_link";
    closed_points.type = visualization_msgs::Marker::POINTS;
    closed_points.points.clear();
    closed_points.color.g = 1;
    closed_points.color.a = 1;
    geometry_msgs::Point point;
    for(int i = 0; i < c_size; i++){
      point.x = milestones(0,closed(0,i));
      point.y = milestones(1,closed(0,i));
      closed_points.points.push_back(point);
    }
  marker_publisher.publish(closed_points);
  }
  geometry_msgs::Point point;
    for(int i = 0; i < c_size; i++){
      point.x = milestones(0,closed(0,i));
      point.y = milestones(1,closed(0,i));
      closed_points.points.push_back(point);
    }
  marker_publisher.publish(closed_points);
  done = 0;
  double cur = end;
  int curC;
  for(int i=0;i<c_size;i++){
    if(closed(0,i)==end){
      curC = i;
    }
  }
  double prev = closed(1,curC);
  Eigen::ArrayXd spath;
  int spath_size = 1;
  spath.conservativeResize(spath_size);
  spath(spath_size-1) = cur;
  while(!done){
    if(prev == start){
      done = 1;
    }
    cur = prev;
    for(int i=0;i<c_size;i++){
      if(closed(0,i)==cur){
        curC = i;
      }
    }
    prev = closed(1,curC);
    spath_size += 1;
    spath.conservativeResize(spath_size);
    spath(spath_size-1) = cur;
  }
  visualization_msgs::Marker line_list2;
  line_list2.id = 3;
  line_list2.header.frame_id = "/base_link";
  line_list2.type = visualization_msgs::Marker::LINE_LIST;
  line_list2.scale.x = 0.05;
  line_list2.scale.y = 0.05;
  line_list2.scale.z = 0.01;
  line_list2.color.b = 1.0;
  line_list2.color.a = 1.0;
  geometry_msgs::Point p;
  line_list2.points.clear();
  for(int i=1;i<spath_size;i++){
    p.x = milestones(0,spath(i-1));
    p.y = milestones(1,spath(i-1));
    line_list2.points.push_back(p);
    p.x = milestones(0,spath(i));
    p.y = milestones(1,spath(i));
    line_list2.points.push_back(p);
  }
  marker_publisher.publish(line_list2);
  return spath;
}
bool interference(int x, int y, Eigen::MatrixXd &map, int mapX, int mapY, int buffer){
  int lowX, lowY, highX, highY;
  if(x - buffer < 0){
    lowX = 0;
  } else {
    lowX = x - buffer;
  }
  if(y - buffer < 0){
    lowY = 0;
  } else {
    lowY = y - buffer;
  }
  if(x + buffer > mapX){
    highX = mapX;
  } else {
    highX = x + buffer;
  }
  if(y + buffer > mapY){
    highY = mapY;
  } else {
    highY = y + buffer;
  }
  
  for(int i=lowX;i<highX;i++){
    for(int j=lowY;j<highY;j++){
      if(map(j,i)){
        return true;
      }
    }
  }
return false; 
}
bool collision(int x1, int y1, int x2, int y2, Eigen::MatrixXd &map, int buffer){
  x.clear();
  y.clear();
  bresenham(10*x1,10*y1,10*x2,10*y2,x,y);
  for (int i=0;i<x.size();i++){
    if(interference(x[i],y[i],map,100,100,buffer)){
      return true;
    }
  }
return false;
}
Eigen::MatrixXf GenerateProbabilisticRoadMap(Eigen::MatrixXd &obstacleMap, int numNodes, Eigen::MatrixXf start, Eigen::MatrixXf end){
  Eigen::MatrixXf points(3,numNodes);  
  int mapWidth = obstacleMap.rows();
  int mapLength = obstacleMap.cols();
  double dist;
  double buffer = 3;
  std::default_random_engine generator;
  std::uniform_real_distribution<double> distribution(0.0,10.0);
  int numObstacles = 0;
  int numMilestones = numNodes;
  visualization_msgs::Marker line_list;
  points(0,0) = start(0);
  points(1,0) = start(1);
  points(2,0) = 0;
  points(0,1) = end(0);
  points(1,1) = end(1);
  points(2,1) = 0; //Add start and end nodes to set
  for(int i = 2; i < numNodes; i++){ //Generate nodes at random
    points(0,i) = distribution(generator);
    points(1,i) = distribution(generator);
    points(2,i) = 0;
  }
  // check node interference
  for (int i=0;i<numNodes;i++){
    if(interference(round(10*points(0,i)),round(10*points(1,i)),obstacleMap,100,100,buffer)){
      points(2,i) = 1;
      numMilestones -= 1;
    }
  }
  //Create milestones list
  Eigen::MatrixXf milestones(2,numMilestones); //+2 because need to add start and end points
  int index = 0;
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
  int numConnections = 10; //Number of closest points to look at
  //Connect milestones
  for(int i=0;i<numMilestones;i++){
    for(int j=0;j<numMilestones;j++){
      dists(j) = get_dist(milestones(0,i),milestones(1,i),milestones(0,j),milestones(1,j));
    }
    sorted_dists = sort(dists);
    for(int k=0;k<numConnections;k++){
      //Keep in mind: i represents the current milestone
      // if (!check_collision(milestones(0,i),milestones(1,i),milestones(0,sorted_dists(1,k)),milestones(1,sorted_dists(1,k)),obstacleCoords,numObstacles,buffer) && (i != sorted_dists(1,k))){
      if(!collision(round(milestones(0,i)),round(milestones(1,i)),round(milestones(0,sorted_dists(1,k))),round(milestones(1,sorted_dists(1,k))),obstacleMap,buffer)){
        connections(i,sorted_dists(1,k)) = 1;
        connections(sorted_dists(1,k),i) = 1; //If i is connected to j, then j is connected to i
      }
    }
  }
  line_list.id = 1;
  line_list.header.frame_id = "/base_link";
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.scale.y = 0.01;
  line_list.scale.z = 0.01;
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
  marker_publisher.publish(line_list);
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;
  marker.id = 0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.header.frame_id = "/base_link";
  marker.type = visualization_msgs::Marker::POINTS;
  marker.points.clear();
  marker.colors.clear();
  for(int i = 0; i < numNodes; i++){
    p.x = points(0,i);
    p.y = points(1,i);
    color.r = points(2,i);
    color.g = points(2,i);
    color.b = points(2,i);
    color.a = 1;
    marker.points.push_back(p);
    marker.colors.push_back(color);
  }
  marker_publisher.publish(marker);
  Eigen::ArrayXd spath;
  spath = a_star(milestones, connections, 0, 1, numMilestones);
  Eigen::MatrixXf waypoints(2, spath.size());
  for(int i=0;i<spath.size();i++){
    waypoints(0,i) = milestones(0,spath(spath.size()-1-i));
    waypoints(1,i) = milestones(1,spath(spath.size()-1-i));
  }
  return waypoints;
}

double correctTo2Pi(double angle){
  double newAngle = angle;
  if(angle<0){
    newAngle += 2*PI;
  }
  else if(angle > 2*PI){
    newAngle -= 2*PI;
  }else{
    //ROS_INFO("PI Correction Not Needed");
  }
  return newAngle;
}

void determineNewHeading(double nextX, double nextY, double &linearSpeed, double &turnSpeed){
      
      
      double desiredTheta = 0; //robot to point angle
      double difference = 0;
      double largeAngle = 30*3.1415/180; //30 degrees
      int heading = 0; //rads      
      double rotSpeed = 0.2;
      double linVel = 0.1;
      int limit = 0.75;
      double thetaOverflowCheck = 0;
      double desiredOverflowCheck = 0;      
      float dY = 0;
      float dX = 0;
      float tolerance = 5*PI/180;
      dY = nextY-YPos;
      dX = nextX-XPos;
      ROS_INFO("dy %f", dY);
      ROS_INFO("dx %f", dX);
      desiredTheta = atan(dY/dX);
      desiredTheta = correctTo2Pi(desiredTheta);    
      
      ROS_INFO("th is %f", desiredTheta);
      ROS_INFO("theta is %f", theta);

      difference = desiredTheta - theta;
      ROS_INFO("difference: %f", difference);

      thetaOverflowCheck = correctTo2Pi(theta+PI);
      desiredOverflowCheck = correctTo2Pi(desiredTheta+PI);

      rotSpeed = 0.3*abs(difference); //the larger the difference the greater the rotation speed
      linearSpeed = 0.05;

      if(theta >= PI && thetaOverflowCheck > desiredTheta){ //Overflow case, theta in quadrant 3/4 and desired theta in quadrant 1, faster to turn left
        ROS_INFO("1");
        turnSpeed = rotSpeed; //positive is a left turn
        linVel = 0;              
      }
      else if(desiredTheta >= PI && desiredOverflowCheck > theta){ //Overflow case 2, desired theta in quadrant 3/4 and theta in quadrant 1, faster to turn right        
         ROS_INFO("2");
         turnSpeed = -rotSpeed;
         linVel = 0;                                   
      }
      else if(abs(difference) < tolerance){ //want it to go straight within a certain tolerance, just not sure what that tolerance should be. 5 degrees first guess.
        ROS_INFO("3");
        turnSpeed = 0;
        linVel = 0.1;        
      }
      else if(difference > 0 + tolerance){
        ROS_INFO("4");
        turnSpeed = rotSpeed;
        linVel = 0;
      }
      else if(difference < 0 - tolerance){
        ROS_INFO("5");
        turnSpeed = -rotSpeed;
        linVel = 0;
      }else{
        ROS_INFO("6");
      }
      linearSpeed = linVel;
         
}

void wait(){
  for(int i = 0; i < 5*100000; i++);
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
    Eigen::MatrixXf path;
    Eigen::MatrixXf waypoints;
    

    int wp_size1 = 0;
    int wp_size2 = 0;
    //Set the loop rate
    ros::Rate loop_rate(20);    //20Hz update rate
    int numNodes = 500;
    int cWP; //current waypoint
    double th;
    int prmComplete = 0;
    double dY;
    double dX;
    float limit = 0.6;
    double linearSpeed = 0;
    double angularSpeed = 0;
    double nextPointY = 0;
    double nextPointX = 0;
    Eigen::MatrixXf start(2,1);
    Eigen::MatrixXf goals(2,3);
    start(0,0) = 1;
    start(1,0) = 5;
    goals(0,0) = 5;
    goals(1,0) = 5;
    goals(0,1) = 9;
    goals(1,1) = 1;
    goals(0,2) = 9;
    goals(1,2) = 5;
    int ind = 0;  

    while(!mapComplete){
      loop_rate.sleep();
      ros::spinOnce();
    }
    while(mapComplete && prmComplete != 3){
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();
      ROS_INFO("%d", prmComplete);
      if(mapComplete && prmComplete != 3){
        path = GenerateProbabilisticRoadMap(map,numNodes,start,goals.col(prmComplete));
        wp_size2 = wp_size1 + path.cols();
        waypoints.conservativeResize(2,wp_size2);
        for(int i=wp_size1;i<wp_size2;i++){
          waypoints(0,i) = path(0,ind);
          waypoints(1,i) = path(1,ind);
          ind++;
        }
        ind = 0;
        wp_size1 = wp_size2;
        start = goals.col(prmComplete);
        prmComplete++;
        ROS_INFO("prmComplete: %d",prmComplete);
      }
      else {
        ROS_INFO("path size: %ld",waypoints.cols());
        for(int i=0;i<waypoints.cols();i++){
          //ROS_INFO("path(0,%d): %f, path(1,%d): %f",i,cPath(0,i),i,cPath(1,i));
          //Need to map waypoints to gazebo coordinate system
          ROS_INFO("transformed: path(0,%d): %f, path(1,%d): %f",i,waypoints(0,i)-1,i,waypoints(1,i)-5);
        }
        ROS_INFO("path found");
        visualization_msgs::Marker line_list3;
        line_list3.id = 4;
        line_list3.header.frame_id = "/base_link";
        line_list3.type = visualization_msgs::Marker::LINE_LIST;
        line_list3.scale.x = 0.05;
        line_list3.scale.y = 0.05;
        line_list3.scale.z = 0.01;
        line_list3.color.g = 1.0;
        line_list3.color.a = 1.0;
        geometry_msgs::Point p;
        line_list3.points.clear();
        for(int i=1;i<waypoints.cols();i++){
          p.x = waypoints(0,i-1);
          p.y = waypoints(1,i-1);
          line_list3.points.push_back(p);
          p.x = waypoints(0,i);
          p.y = waypoints(1,i);
          line_list3.points.push_back(p);
        }
        marker_publisher.publish(line_list3);
      }
    }
    Eigen::MatrixXf remappedWaypoints(waypoints.rows(),waypoints.cols());
    for(int k = 0; k < waypoints.cols(); k++){

          remappedWaypoints(0,k) = waypoints(0,k)-1;
          remappedWaypoints(1,k) = waypoints(1,k)-5;
          ROS_INFO("X: %lf ,Y: %lf ",remappedWaypoints(0,k),remappedWaypoints(1,k));
    }
    wait();
    wait();


    int wayPointsIndex = 1;
    double dist = 0;
    while (ros::ok()) //Main Loop
    {
      loop_rate.sleep(); //Maintain the loop rate
      ros::spinOnce();   //Check for new messages      
      
      if(wayPointsIndex <= waypoints.cols()-1){
          //ROS_INFO("XPOS %f", XPos);
          //ROS_INFO("YPOS %f", YPos);

          nextPointY = remappedWaypoints(0,wayPointsIndex);//I will never understand the X Y mapping
          nextPointX = remappedWaypoints(1,wayPointsIndex);
          ROS_INFO("Y Pos %f", YPos);
          ROS_INFO("X Pos %f", XPos);
          dist = get_dist(XPos, YPos, nextPointX, nextPointY);
          ROS_INFO("xPoint %f", nextPointX);
          ROS_INFO("yPoint %f", nextPointY);
          ROS_INFO("dist %f", dist);
          ROS_INFO("rotation %f", angularSpeed);

          if(dist > limit){
            determineNewHeading(nextPointX, nextPointY, linearSpeed, angularSpeed);            
            
            //ROS_INFO("linearSpeed %f", linearSpeed);
            //ROS_INFO("angleSpeed %f", angularSpeed);

            vel.linear.x = linearSpeed;
            vel.angular.z = angularSpeed;
          }
          else{
            ROS_INFO("Y Point %f", nextPointY);
            ROS_INFO("X Point %f", nextPointX);
            wait();
            wayPointsIndex++;
            ROS_INFO("index %f", wayPointsIndex);
          }

      }
      else{
          vel.linear.x = 0;
          vel.angular.z = 0;          
      } 
      
      velocity_publisher.publish(vel);
      /*
      vel.angular.z = -0.5; //turns right
      velocity_publisher.publish(vel);
      for(int i = 0; i < 5*10000; i++){
        ROS_INFO("%d \n",-1*i);
      };

      vel.angular.z = 0;
      velocity_publisher.publish(vel);
      for(int i = 0; i < 10000; i++){
        ROS_INFO("Waiting");
      }

      vel.angular.z = 0.5; //turns left
      velocity_publisher.publish(vel);
      for(int i = 0; i < 5*10000; i++){
        ROS_INFO("%d \n",i);
      };  */
          //followWaypoints(waypoints,vel,velocity_publisher);          
        
      // else{
      //   visualization_msgs::Marker line_list3;
      //   line_list3.id = 4;
      //   line_list3.header.frame_id = "/base_link";
      //   line_list3.type = visualization_msgs::Marker::LINE_LIST;
      //   line_list3.scale.x = 0.05;
      //   line_list3.scale.y = 0.05;
      //   line_list3.scale.z = 0.01;
      //   line_list3.color.b = 1.0;
      //   line_list3.color.a = 1.0;
      //   geometry_msgs::Point p;
      //   line_list3.points.clear();
      //   for(int i=0;i<paths.rows();i++){
      //     for(int j=0;i<paths.cols();j++){
      //       p.x = milestones(0,spath(i-1));
      //       p.y = milestones(1,spath(i-1));
      //       line_list2.points.push_back(p);
      //       p.x = milestones(0,spath(i));
      //       p.y = milestones(1,spath(i));
      //       line_list2.points.push_back(p);
      //     }
      //     marker_publisher.publish(line_list3);
      //   }
      // }
      ////////////// This is my bad path following code, you can ignore it
      // dY = waypoints(1,cWP) - YPos;
      // dX = waypoints(0,cWP) - XPos;
      // th = atan(dY/dX);
      // if(th < 0 && dX < 0 && dY > 0){
      //   th = -th;
      //   ROS_INFO("th < 0");
      // }
      // else if(th < 0 && dX > 0 && dY < 0){
      //   th = th + 2*M_PI;
      //   ROS_INFO("th < 0");
      // }
      // if(th > 2*M_PI){
      //   th = th - 2*M_PI;
      //   ROS_INFO("th > 2pi");
      // }
      // if(fabs(th-theta)<0.01){
      //   vel.angular.z = 0;
      //   vel.linear.x = 0.15;
      //   ROS_INFO("Moving to: %f, %f",waypoints(0,cWP),waypoints(1,cWP));
      //   ROS_INFO("current X: %f, current Y: %f, current th: %f, target th: %f",XPos, YPos,theta,th);
      //   if(get_dist(XPos,YPos,waypoints(0,cWP),waypoints(1,cWP))<0.05){
      //     cWP -= 1; //Move to next waypoint;
      //     ROS_INFO("next waypoint");
      //   }
      // } else {
      //   if( theta > th && (theta - th) < M_PI ){
      //     vel.angular.z = -0.3;
      //   } 
      //   else if( theta < th && (th - theta) > M_PI) {
      //     vel.angular.z = -0.3;
      //   }
      //   else if( theta < th && (th - theta) < M_PI){
      //     vel.angular.z = 0.3;
      //   }
      //   else if( theta > th && (theta - th) > M_PI){
      //     vel.angular.z = 0.3;
      //   }
      //   vel.linear.x = 0;
      //   ROS_INFO("Moving to: %f rads, currently at %f rads",th,theta);
      // }
      // velocity_publisher.publish(vel);
      // ROS_INFO("WP: %d",cWP);
      /////////////////
    }
    return 0;
} 