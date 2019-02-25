#include <vector>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include "FORCESNLPsolver.h"
#include "matplotlibcpp.h"
#include <cmath>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <multidrone_msgs/DirectorEvent.h>
#include <nav_msgs/Odometry.h>


///////// solver params /////////

const int time_horizon = 30; // time horizon
const int n_states_variables = 9;

const float DESIRED_WAYPOINT_X = 3.35;
const float DESIRED_WAYPOINT_Y = -44;
const float DESIRED_WAYPOINT_Z = 4;

// no fly zones
const float POS_OBSTACLE_X = -1.15;
const float POS_OBSTACLE_Y = -36.3;  
const float POS_OBSTACLE_Z = 10;


const float hovering_distance = 0.5;


// state vector

const int acceleration_x = 0;
const int acceleration_y = 1;
const int acceleration_z = 2;

const int position_x = 3;
const int position_y = 4;
const int position_z = 5;

const int velocity_x = 6;
const int velocity_y = 7;
const int velocity_z = 8;


// log

std::ofstream csv_ual; // logging the trayectory
std::ofstream csv_pose; // logging the pose


void init();
bool solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z,std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz, float desired_wp_x, float desired_wp_y, float desired_wp_z, float desired_vel_x, float desired_vel_y, float desired_vel_z,float obst_x, float obst_y, float obst_z);
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
void logToCsv(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz, int n_steps);


