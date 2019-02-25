#include "FORCESNLPsolver.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "uav_path_manager/GeneratePath.h"
#include "uav_path_manager/GetGeneratedPath.h"
#include <thread>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <optimal_control_interface.h>

geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped own_pose;
bool position_control;
ros::Publisher set_pose_pub;
ros::Publisher set_velocity_pub;

std::vector<double> x;
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;
std::vector<double> x_ual;
std::vector<double> y_ual;
std::vector<double> z_ual;
std::vector<double> vx_ual;
std::vector<double> vy_ual;
std::vector<double> vz_ual;


bool solver_success;

/** Callback for the target pose
 */

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    target_pose.pose = msg->pose.pose;
}


void UALthread(){
    
    int cont = 0;
    ros::Rate rate(10);
    ROS_INFO("thread initialized");

    if(solver_success) cont = 10;
    else cont = cont+1;
       
    /***************** POSE ****************************/
    if(position_control){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x_ual[cont];
        pose.pose.position.y = y_ual[cont];
        pose.pose.position.z = z_ual[cont];
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        // yaw pointing to the path
        double dx = x_ual[cont] - own_pose.pose.position.x;
        double dy = y_ual[cont] - own_pose.pose.position.y;
        double yaw = atan2(dy,dx);
        geometry_msgs::Quaternion quat = toQuaternion(0.0, 0.0, yaw);
        pose.pose.orientation = quat;
        set_pose_pub.publish(pose);
    }
    else{
         /************** VELOCITY ***********************/
        geometry_msgs::TwistStamped vel;
        vel.twist.linear.x = vx_ual[cont];
        vel.twist.linear.y = vy_ual[cont];
        vel.twist.linear.z = vz_ual[cont];
        vel.header.frame_id = "map";
        vel.header.stamp = ros::Time::now();
        set_velocity_pub.publish(vel);
        // TODO controlling yaw
    }
    
    ros::spinOnce();
    rate.sleep();
    

}

int main(int _argc, char **_argv)
{
    int n_steps;
    ros::init(_argc, _argv, "optimal_control_interface_node");
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");

    float height_take_off;
    // parameters
    pnh.param<float>("height_take_off", height_take_off, 4.0);
    pnh.param<bool>("position_control", position_control, false);
    pnh.param<int>("n_steps_control", n_steps, 10);
    // subscribers and publishers

    ros::Subscriber target_pose_sub = nh.subscribe<nav_msgs::Odometry>("drc_vehicle_xp900/odometry", 1, targetPoseCallback);
   

    ros::ServiceClient go_to_waypoint_srv_ = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("uav_1/ual/go_to_waypoint");
    ros::ServiceClient take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("drone_1/ual/take_off");

    set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_1/ual/set_pose",1);
    set_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone_1/ual/set_velocity",1);

    // init solver
    init();
    // taking off

    ROS_INFO("taking off");
    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = height_take_off;
    take_off_srv.call(srv);

    // thread for calling the UAL
    std::thread threadObjSolver(UALthread);

    while(ros::ok){
        ros::Rate rate(1);//hz
          // calling solver
        ROS_INFO("solver running");
   
        // solver function
        float desired_wp_x, desired_wp_y, desired_wp_z, desired_vel_x, desired_vel_y, desired_vel_z, obst_x, obst_y, obst_z;
        solver_success = solverFunction(x,y,z,vx,vy,vz,desired_wp_x, desired_wp_y, desired_wp_z, desired_vel_x, desired_vel_y, desired_vel_z,obst_x, obst_y, obst_z);
        if(solver_success){
            x_ual.clear();
            y_ual.clear();
            z_ual.clear();
            vx_ual.clear();
            vy_ual.clear();
            vz_ual.clear();
            x_ual = x;
            y_ual = y;
            z_ual = z;
            vx_ual = vx;
            vy_ual = vy;
            vz_ual = vz;
        }

        logToCsv(x,y,z,vx,vy,vz,n_steps);

        rate.sleep();
        ros::spinOnce();
    }
    
    threadObjSolver.join();
    return 0;
}
