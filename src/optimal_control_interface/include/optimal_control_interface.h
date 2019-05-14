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
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <multidrone_msgs/DirectorEvent.h>
#include <nav_msgs/Odometry.h>
#include <time.h> 

// global vars

geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped own_pose;
geometry_msgs::TwistStamped own_velocity;
ros::ServiceClient go_to_waypoint_srv_;
ros::Subscriber sub_position;
ros::Subscriber sub_velocity;
ros::Publisher path_rviz_pub;
ros::Publisher path_no_fly_zone;
ros::Publisher desired_pose_publisher;
clock_t initial_time;

///////// solver params /////////

const int time_horizon = 100; // time horizon
const int n_states_variables = 11;
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

// initial guess

const float u_x = 0.5;
const float u_y = 0.5;
const float u_z = 0.5;

const float p_x = 2;
const float p_y = 3;
const float p_z = 2;

const float v_x = 1;
const float v_y = 1;
const float v_z = 2;

const float t_x = 1;
const float t_y = 1;
//const float p_x_previous = 1;
//const float p_y_previous = 0.5;
//const float p_z_previous = 3;

//const float v_x_previous = 2;
//const float v_y_previous = 1;
//const float v_z_previous = 1;


// shooting actions types

const int flyover = 0;
const int orbital = 1;
const int lateral = 2;

// log

std::ofstream csv_ual; // logging the pose
std::ofstream csv_pose; // logging the trajectory


void init(ros::NodeHandle nh);
bool solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,std::vector<double> &desired_wp, std::vector<double> desired_vel, std::vector<double> &obst);
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
void logToCsv(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz, int n_steps);
void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z, std::vector<double> &desired_wp);
void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]);
void publishDesiredPoint(double x, double y,double z);
void calculateDesiredPoint(int shooting_type, const std::vector<double> &target_pose, std::vector<double> &desired_pose, std::vector<double> desired_velocity);
