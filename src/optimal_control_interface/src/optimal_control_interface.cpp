/** ros node to interface with solver created by FORCES PRO */
#include <ros/ros.h>
#include "FORCESNLPsolver.h"
#include "matplotlibcpp.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <thread>
#include <nav_msgs/Path.h>
#include <boost/thread/thread.hpp>
#include <uav_abstraction_layer/TakeOff.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include "uav_path_manager/GeneratePath.h"
#include "uav_path_manager/GetGeneratedPath.h"
#include <Eigen/Eigen>
#include <multidrone_msgs/DirectorEvent.h>

#include <optimal_control_interface.h>


geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped own_pose;
geometry_msgs::TwistStamped own_velocity;
ros::ServiceClient go_to_waypoint_srv_;
ros::Publisher set_pose_pub; 
ros::Publisher set_velocity_pub;
ros::ServiceClient client_follower, client_generator;
ros::Subscriber sub_velocity;
std::string event_received_id;
bool event_received = false;
ros::ServiceClient take_off_srv;





int offset = 10;

#ifdef __cplusplus
extern "C"
{
#endif
    extern void FORCESNLPsolver_casadi2forces(double *x, double *y, double *l, double *p,
                          double *f, double *nabla_f, double *c, double *nabla_c,
                          double *h, double *nabla_h, double *H, int stage);
#ifdef __cplusplus
}
#endif
namespace plt = matplotlibcpp;

/////////////////////////////////// CALLBACKS //////////////////////////////////////////////

/** Callback for the target pose
 */

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    target_pose.pose = msg->pose.pose;
}

/** Drone pose callback
 */
void ownPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    own_pose.pose = msg->pose;
    csv_pose << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << std::endl;
}
/** Drone velocity callback
 */
void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    own_velocity.twist = msg->twist;
}

///////////////// UTILITY FUNCTIONS ////////////////////

/** Utility function for hovering
 */

bool checkHovering(){
    Eigen::Vector3f current_pose, current_target_pose;
    current_pose = Eigen::Vector3f(own_pose.pose.position.x, own_pose.pose.position.y, own_pose.pose.position.z);
    current_target_pose = Eigen::Vector3f(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    if((current_target_pose - current_pose).norm() > hovering_distance) return false;
    else return true;
}

/** Utility function to get quaternion from pitch, roll, yaw
 */
geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
	geometry_msgs::Quaternion q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}

bool directorEventCb(multidrone_msgs::DirectorEvent::Request &req,
                     multidrone_msgs::DirectorEvent::Request &res)
{
  event_received_id = req.event_id;
  event_received = true;
  ROS_INFO("event %s received from the Director", event_received_id.c_str());
  return true;
}

/** Utility function for plotting the result through matplotlib
 */

void plottingResult(FORCESNLPsolver_output *myoutput){

    plt::plot(x, y, "b");
    plt::xlim(6.0, 11.0);
    plt::ylim(6.0, 6.0);
    plt::show();

}

/** function for logging to csv file
 */



/**  Construct a nav_msgs_path
 */

nav_msgs::Path constructNavMsgsPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
    nav_msgs::Path msg;
    std::vector<geometry_msgs::PoseStamped> poses(wps_x.size());
    msg.header.frame_id = "map";
    for (int i = 0; i < wps_x.size(); i++) {
        poses.at(i).pose.position.x = wps_x[i];
        poses.at(i).pose.position.y = wps_y[i];
        poses.at(i).pose.position.z = wps_z[i];
        poses.at(i).pose.orientation.x = 0;
        poses.at(i).pose.orientation.y = 0;
        poses.at(i).pose.orientation.z = 0;
        poses.at(i).pose.orientation.w = 1;
    }
    msg.poses = poses;
    return msg;
}


//////////////////////////////////////////////////////////////

/** Control function. This function interfaces the MPC with UAL
 */

void UALthread(){
    int cont = 0;
    sleep(5);

    ros::Rate rate(10);//hz
    ROS_INFO("thread initialized");

     while(event_received_id!="GET_READY"){
        sleep(0.5);
    }
    ROS_INFO("taking off");
    // taking off
    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = 5;
    take_off_srv.call(srv);

    // wait for start race
    while(event_received_id != "START_RACE" ){
        sleep(0.5);
    }
    /////////////////////////////////////////////////////////////
    while(event_received_id !="FORCES"){
        if(solver_called) cont = 10;
        else cont = cont+1;
        // call the UAL
        /*uav_abstraction_layer::GoToWaypoint pose_srv;
        pose_srv.request.waypoint.pose.position.x = x[cont];
        pose_srv.request.waypoint.pose.position.y = y[cont];
        pose_srv.request.waypoint.pose.position.z = z[cont];
        std::cout<<"pose x: "<<x[cont]<<" y: "<<y[cont]<<" z: "<<z[cont]<<std::endl;
        pose_srv.request.waypoint.pose.orientation = target_pose.pose.orientation;
        pose_srv.request.waypoint.header.frame_id = "map";
        pose_srv.request.waypoint.header.stamp = ros::Time::now();
        pose_srv.request.blocking = false;
        */
        /***************** POSE ****************************/
         geometry_msgs::PoseStamped pose;
       
        pose.pose.position.x = DESIRED_WAYPOINT_X;
        pose.pose.position.y = DESIRED_WAYPOINT_Y;
        pose.pose.position.z = DESIRED_WAYPOINT_Z;
        pose.pose.orientation = target_pose.pose.orientation;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        set_pose_pub.publish(pose); 
        /*
        
        /************* YAW *****************************/
       /* double dx = x[cont] - own_pose.pose.position.x;
        double dy = y[cont] - own_pose.pose.position.y;
        double yaw = atan2(dy,dx);
        geometry_msgs::Quaternion quat = toQuaternion(0.0, 0.0, yaw);

        pose.pose.orientation = quat;

        /************** VELOCITY ***********************/


       /* geometry_msgs::TwistStamped vel;
        vel.twist.linear.x = vx[cont];
        vel.twist.linear.y = vy[cont];
        vel.twist.linear.z = vz[cont];
        
        for(int i=0; i<vx.size();i++){
            std::cout<<vx[i]<<std::endl;
        }
        vel.header.frame_id = "map";
        vel.header.stamp = ros::Time::now();
        set_velocity_pub.publish(vel);*/

       /* if(!checkHovering()) set_velocity_pub.publish(velocity);
        else{
        geometry_msgs::TwistStamped vel;
        vel.twist.linear.x = 0;
        vel.twist.linear.y = 0;
        vel.twist.linear.z = 0;
        set_velocity_pub.publish(velocity);
        } */

        
        
      //  cont = cont+1;
        ros::spinOnce();
        rate.sleep();
    }
}

/////////////////////////////////////////////////////////////////////////////

/** utility function to save the output
 */
void saveOutput(int state_variable){

}


/** solver function
*/

void solverFunction(){

       // solver variables
    /* declare FORCES variables and structures */
    FORCESNLPsolver_info myinfo;
    FORCESNLPsolver_params myparams;
    FORCESNLPsolver_output myoutput;
   
    /* define external function evaluating functions and derivatives (only for the high-level interface) */

    FORCESNLPsolver_extfunc pt2Function = &FORCESNLPsolver_casadi2forces;

    int i, exitflag;
    /* fill the params */
    myparams.xinit[0] = own_pose.pose.position.x;
    myparams.xinit[1] = own_pose.pose.position.y;
    myparams.xinit[2] = own_pose.pose.position.z;
    myparams.xinit[3] = own_velocity.twist.linear.x;
    myparams.xinit[4] = own_velocity.twist.linear.y;
    myparams.xinit[5] = own_velocity.twist.linear.z;

    // set initial guess
    std::vector<double> x0;
    double x0i[] = {0, 0, 0, 0, 0, 15, 0.5, 0.5, 0.5};
    for (int j = 0; j < time_horizon; j++)
    {
        for (i = 0; i < n_states_variables; i++)
        {
            x0.push_back(x0i[i]);
        }
    }

    for (i = 0; i < x0.size(); i++)
    {
        myparams.x0[i] = x0[i];
    }
    // set parameters
    std::vector<double> params;
    double def_param[] = {DESIRED_WAYPOINT_X, DESIRED_WAYPOINT_Y, DESIRED_WAYPOINT_Z,
                        own_velocity.twist.linear.x, own_velocity.twist.linear.y, own_velocity.twist.linear.z,
                        POS_OBSTACLE_X, POS_OBSTACLE_Y, POS_OBSTACLE_Z};

    for(int i = 0; i<time_horizon; i++){
        for(int j=0; j<9; j++){
            params.push_back(def_param[j]);
        }
    }
    for(int i=0; i<params.size();i++) 
    {
        myparams.all_parameters[i] = params[i];
    }
     // call the solver
    
    exitflag = FORCESNLPsolver_solve(&myparams, &myoutput, &myinfo, stdout, pt2Function);
    solver_called = true;
    // save the output in a vector
    x.clear();
    y.clear();
    z.clear();

    if(exitflag == 1){  // if if the call to the solver is successful, save the output
        x.push_back(myoutput.x01[position_x]);
        x.push_back(myoutput.x02[position_x]);
        x.push_back(myoutput.x03[position_x]);
        x.push_back(myoutput.x04[position_x]);
        x.push_back(myoutput.x05[position_x]);
        x.push_back(myoutput.x06[position_x]);
        x.push_back(myoutput.x07[position_x]);
        x.push_back(myoutput.x08[position_x]);
        x.push_back(myoutput.x09[position_x]);
        x.push_back(myoutput.x10[position_x]);
        x.push_back(myoutput.x11[position_x]);
        x.push_back(myoutput.x12[position_x]);
        x.push_back(myoutput.x13[position_x]);
        x.push_back(myoutput.x14[position_x]);
        x.push_back(myoutput.x15[position_x]);
        x.push_back(myoutput.x16[position_x]);
        x.push_back(myoutput.x17[position_x]);
        x.push_back(myoutput.x18[position_x]);
        x.push_back(myoutput.x19[position_x]);
        x.push_back(myoutput.x20[position_x]);
        x.push_back(myoutput.x21[position_x]);
        x.push_back(myoutput.x22[position_x]);
        x.push_back(myoutput.x23[position_x]);
        x.push_back(myoutput.x24[position_x]);
        x.push_back(myoutput.x25[position_x]);
        x.push_back(myoutput.x26[position_x]);
        x.push_back(myoutput.x27[position_x]);
        x.push_back(myoutput.x28[position_x]);
        x.push_back(myoutput.x29[position_x]);
        x.push_back(myoutput.x30[position_x]);

        vx.push_back(myoutput.x01[velocity_x]);
        vx.push_back(myoutput.x02[velocity_x]);
        vx.push_back(myoutput.x03[velocity_x]);
        vx.push_back(myoutput.x04[velocity_x]);
        vx.push_back(myoutput.x05[velocity_x]);
        vx.push_back(myoutput.x06[velocity_x]);
        vx.push_back(myoutput.x07[velocity_x]);
        vx.push_back(myoutput.x08[velocity_x]);
        vx.push_back(myoutput.x09[velocity_x]);
        vx.push_back(myoutput.x10[velocity_x]);
        vx.push_back(myoutput.x11[velocity_x]);
        vx.push_back(myoutput.x12[velocity_x]);
        vx.push_back(myoutput.x13[velocity_x]);
        vx.push_back(myoutput.x14[velocity_x]);
        vx.push_back(myoutput.x15[velocity_x]);
        vx.push_back(myoutput.x16[velocity_x]);
        vx.push_back(myoutput.x17[velocity_x]);
        vx.push_back(myoutput.x18[velocity_x]);
        vx.push_back(myoutput.x19[velocity_x]);
        vx.push_back(myoutput.x20[velocity_x]);
        vx.push_back(myoutput.x21[velocity_x]);
        vx.push_back(myoutput.x22[velocity_x]);
        vx.push_back(myoutput.x23[velocity_x]);
        vx.push_back(myoutput.x24[velocity_x]);
        vx.push_back(myoutput.x25[velocity_x]);
        vx.push_back(myoutput.x26[velocity_x]);
        vx.push_back(myoutput.x27[velocity_x]);
        vx.push_back(myoutput.x28[velocity_x]);
        vx.push_back(myoutput.x29[velocity_x]);
        vx.push_back(myoutput.x30[velocity_x]);
        // xplot.push_back(myoutput->x31[3]);
        // xplot.push_back(myoutput->x32[3]);
        // xplot.push_back(myoutput->x33[3]);
        // xplot.push_back(myoutput->x34[3]);
        // xplot.push_back(myoutput->x35[3]);
        // xplot.push_back(myoutput->x3velocity_x[3]);
        // xplot.push_back(myoutput->x37[3]);
        // xplot.push_back(myoutput->x38[3]);
        // xplot.push_back(myoutput->x39[3]);
        // xplot.push_back(myoutput->x40[3]);
        // xplot.push_back(myoutput->x41[3]);
        // xplot.push_back(myoutput->x42[3]);
        // xplot.push_back(myoutput->x43[3]);
        // xplot.push_back(myoutput->x44[3]);
        // xplot.push_back(myoutput->x45[3]);
        // xplot.push_back(myoutput->x4velocity_x[3]);
        // xplot.push_back(myoutput->x47[3]);
        // xplot.push_back(myoutput->x48[3]);
        // xplot.push_back(myoutput->x49[3]);
        // xplot.push_back(myoutput->x50[3]);
        y.push_back(myoutput.x01[position_y]);
        y.push_back(myoutput.x02[position_y]);
        y.push_back(myoutput.x03[position_y]);
        y.push_back(myoutput.x04[position_y]);
        y.push_back(myoutput.x05[position_y]);
        y.push_back(myoutput.x06[position_y]);
        y.push_back(myoutput.x07[position_y]);
        y.push_back(myoutput.x08[position_y]);
        y.push_back(myoutput.x09[position_y]);
        y.push_back(myoutput.x10[position_y]);
        y.push_back(myoutput.x11[position_y]);
        y.push_back(myoutput.x12[position_y]);
        y.push_back(myoutput.x13[position_y]);
        y.push_back(myoutput.x14[position_y]);
        y.push_back(myoutput.x15[position_y]);
        y.push_back(myoutput.x16[position_y]);
        y.push_back(myoutput.x17[position_y]);
        y.push_back(myoutput.x18[position_y]);
        y.push_back(myoutput.x19[position_y]);
        y.push_back(myoutput.x20[position_y]);
        y.push_back(myoutput.x21[position_y]);
        y.push_back(myoutput.x22[position_y]);
        y.push_back(myoutput.x23[position_y]);
        y.push_back(myoutput.x24[position_y]);
        y.push_back(myoutput.x25[position_y]);
        y.push_back(myoutput.x26[position_y]);
        y.push_back(myoutput.x27[position_y]);
        y.push_back(myoutput.x28[position_y]);
        y.push_back(myoutput.x29[position_y]);
        y.push_back(myoutput.x30[position_y]);


        vy.push_back(myoutput.x01[velocity_y]);
        vy.push_back(myoutput.x02[velocity_y]);
        vy.push_back(myoutput.x03[velocity_y]);
        vy.push_back(myoutput.x04[velocity_y]);
        vy.push_back(myoutput.x05[velocity_y]);
        vy.push_back(myoutput.x06[velocity_y]);
        vy.push_back(myoutput.x07[velocity_y]);
        vy.push_back(myoutput.x08[velocity_y]);
        vy.push_back(myoutput.x09[velocity_y]);
        vy.push_back(myoutput.x10[velocity_y]);
        vy.push_back(myoutput.x11[velocity_y]);
        vy.push_back(myoutput.x12[velocity_y]);
        vy.push_back(myoutput.x13[velocity_y]);
        vy.push_back(myoutput.x14[velocity_y]);
        vy.push_back(myoutput.x15[velocity_y]);
        vy.push_back(myoutput.x16[velocity_y]);
        vy.push_back(myoutput.x17[velocity_y]);
        vy.push_back(myoutput.x18[velocity_y]);
        vy.push_back(myoutput.x19[velocity_y]);
        vy.push_back(myoutput.x20[velocity_y]);
        vy.push_back(myoutput.x21[velocity_y]);
        vy.push_back(myoutput.x22[velocity_y]);
        vy.push_back(myoutput.x23[velocity_y]);
        vy.push_back(myoutput.x24[velocity_y]);
        vy.push_back(myoutput.x25[velocity_y]);
        vy.push_back(myoutput.x26[velocity_y]);
        vy.push_back(myoutput.x27[velocity_y]);
        vy.push_back(myoutput.x28[velocity_y]);
        vy.push_back(myoutput.x29[velocity_y]);
        vy.push_back(myoutput.x30[velocity_y]);
        // yplot.push_back(myoutput->x31[4]);
        // yplot.push_back(myoutput->x32[4]);
        // yplot.push_back(myoutput->x33[4]);
        // yplot.push_back(myoutput->x34[4]);
        // yplot.push_back(myoutput->x35[4]);
        // yplot.push_back(myoutput->x36[4]);
        // yplot.push_back(myoutput->x37[4]);
        // yplot.push_back(myoutput->x38[4]);
        // yplot.push_back(myoutput->x39[4]);
        // yplot.push_back(myoutput->x40[4]);
        // yplot.push_back(myoutput->x41[4]);
        // yplot.push_back(myoutput->x42[4]);
        // yplot.push_back(myoutput->x43[4]);
        // yplot.push_back(myoutput->x44[4]);
        // yplot.push_back(myoutput->x45[4]);
        // yplot.push_back(myoutput->x46[4]);
        // yplot.push_back(myoutput->x47[4]);
        // yplot.push_back(myoutput->x48[4]);
        // yplot.push_back(myoutput->x49[4]);
        // yplot.push_back(myoutput->x50[4]);

        vz.push_back(myoutput.x01[velocity_z]);
        vz.push_back(myoutput.x02[velocity_z]);
        vz.push_back(myoutput.x03[velocity_z]);
        vz.push_back(myoutput.x04[velocity_z]);
        vz.push_back(myoutput.x05[velocity_z]);
        vz.push_back(myoutput.x06[velocity_z]);
        vz.push_back(myoutput.x07[velocity_z]);
        vz.push_back(myoutput.x08[velocity_z]);
        vz.push_back(myoutput.x09[velocity_z]);
        vz.push_back(myoutput.x10[velocity_z]);
        vz.push_back(myoutput.x11[velocity_z]);
        vz.push_back(myoutput.x12[velocity_z]);
        vz.push_back(myoutput.x13[velocity_z]);
        vz.push_back(myoutput.x14[velocity_z]);
        vz.push_back(myoutput.x15[velocity_z]);
        vz.push_back(myoutput.x16[velocity_z]);
        vz.push_back(myoutput.x17[velocity_z]);
        vz.push_back(myoutput.x18[velocity_z]);
        vz.push_back(myoutput.x19[velocity_z]);
        vz.push_back(myoutput.x20[velocity_z]);
        vz.push_back(myoutput.x21[velocity_z]);
        vz.push_back(myoutput.x22[velocity_z]);
        vz.push_back(myoutput.x23[velocity_z]);
        vz.push_back(myoutput.x24[velocity_z]);
        vz.push_back(myoutput.x25[velocity_z]);
        vz.push_back(myoutput.x26[velocity_z]);
        vz.push_back(myoutput.x27[velocity_z]);
        vz.push_back(myoutput.x28[velocity_z]);
        vz.push_back(myoutput.x29[velocity_z]);
        vz.push_back(myoutput.x30[velocity_z]);

        z.push_back(myoutput.x01[position_z]);
        z.push_back(myoutput.x02[position_z]);
        z.push_back(myoutput.x03[position_z]);
        z.push_back(myoutput.x04[position_z]);
        z.push_back(myoutput.x05[position_z]);
        z.push_back(myoutput.x06[position_z]);
        z.push_back(myoutput.x07[position_z]);
        z.push_back(myoutput.x08[position_z]);
        z.push_back(myoutput.x09[position_z]);
        z.push_back(myoutput.x10[position_z]);
        z.push_back(myoutput.x11[position_z]);
        z.push_back(myoutput.x12[position_z]);
        z.push_back(myoutput.x13[position_z]);
        z.push_back(myoutput.x14[position_z]);
        z.push_back(myoutput.x15[position_z]);
        z.push_back(myoutput.x16[position_z]);
        z.push_back(myoutput.x17[position_z]);
        z.push_back(myoutput.x18[position_z]);
        z.push_back(myoutput.x19[position_z]);
        z.push_back(myoutput.x20[position_z]);
        z.push_back(myoutput.x21[position_z]);
        z.push_back(myoutput.x22[position_z]);
        z.push_back(myoutput.x23[position_z]);
        z.push_back(myoutput.x24[position_z]);
        z.push_back(myoutput.x25[position_z]);
        z.push_back(myoutput.x26[position_z]);
        z.push_back(myoutput.x27[position_z]);
        z.push_back(myoutput.x28[position_z]);
        z.push_back(myoutput.x29[position_z]);
        z.push_back(myoutput.x30[position_z]);

        //std::cout<<"x: "<<x[30]<< "y: "<<y[30]<<std::endl;

        //
    }
    if (exitflag == 1)
    {
      // print the exitflag for RVIZ    
    }
    else
    {
        //std::cout << "exitflag: " << exitflag << std::endl;
    }
    
}


int main(int _argc, char **_argv)
{

    ros::init(_argc, _argv, "optimal_control_interface_node");
    ros::NodeHandle nh = ros::NodeHandle();
    
    // subscribe to target_pose and gotowaypoint service
    client_generator = nh.serviceClient<uav_path_manager::GeneratePath>("/uav_path_manager/generator/generate_path");
    client_follower = nh.serviceClient<uav_path_manager::GetGeneratedPath>("/uav_path_manager/follower/uav_1/generated_path");
    ros::Subscriber target_pose_sub = nh.subscribe<nav_msgs::Odometry>("drc_vehicle_xp900/odometry", 1, targetPoseCallback);
    ros::Subscriber own_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("drone_1/ual/pose", 1, ownPoseCallback);
    ros::Subscriber own_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("drone_1/ual/velocity", 1, ownVelocityCallback);

    go_to_waypoint_srv_ = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("uav_1/ual/go_to_waypoint");
    take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("drone_1/ual/take_off");

    ros::Publisher path_rviz_pub = nh.advertise<nav_msgs::Path>("/solver/path",1);
    set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_1/ual/set_pose",1);
    set_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone_1/ual/set_velocity",1);
    ros::ServiceServer service = nh.advertiseService("event_manager/director_event", directorEventCb);

    csv_init.open("/home/grvc/init.csv");
    csv_init << std::fixed << std::setprecision(5);

    csv_pose.open("/home/grvc/pose.csv");
    csv_pose << std::fixed << std::setprecision(5);


    // thread for calling the UAL
    std::thread threadObjSolver(UALthread);


    ros::Rate rate(1);//hz
    
    ROS_INFO("solver running");
    while(event_received_id !="FORCES"){
    
        // solver function
        solverFunction();
        /***************** HECTOR ************************ /


        nav_msgs::Path init_path = constructPath(x,y,z);
        // generating a path with more waypoints
        uav_path_manager::GeneratePath generate_path;
        uav_path_manager::GetGeneratedPath give_generated_path;
        std_msgs::Int8 generator_mode;
        std_msgs::Float32 cruising_speed, look_ahead;

        generator_mode.data = 1; //choosing cubic interpolation (1 linear, 2 cubic, 3 cubic-loyal)
        generate_path.request.generator_mode = generator_mode;
        generate_path.request.init_path = init_path;
        client_generator.call(generate_path);
        nav_msgs::Path path = generate_path.response.generated_path;
        give_generated_path.request.generated_path = path;
        cruising_speed.data = 1.5; 
        look_ahead.data = 2.0;
        give_generated_path.request.cruising_speed = cruising_speed;
        give_generated_path.request.look_ahead = look_ahead;
        client_follower.call(give_generated_path);*/
        /********************************************************/


        for(int i=0; i<10; i++){
            csv_init << x[i] << ", " << y[i] << ", " << z[i] << std::endl;
        }



        nav_msgs::Path init_path = constructNavMsgsPath(x,y,z);

        path_rviz_pub.publish(init_path);
        rate.sleep();
        ros::spinOnce();
    }   
    csv_init.close();
    csv_pose.close();

    threadObjSolver.join();
    return 0;
}



