/** ros node to interface with solver created by FORCES PRO */

#include <ros/ros.h>
#include "FORCESNLPsolver.h"
#include <vector>
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



geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped own_pose;
geometry_msgs::TwistStamped own_velocity;
ros::ServiceClient go_to_waypoint_srv_;
ros::Publisher set_pose_pub; 
ros::Publisher set_velocity_pub;
ros::ServiceClient client_follower, client_generator;
ros::Subscriber sub_velocity;
geometry_msgs::TwistStamped velocity;
std::string event_received_id;
bool event_received = false;
ros::ServiceClient take_off_srv;
std::ofstream log2mat


#define WAYPOINT_X -4.38
#define WAYPOINT_Y -39.52
#define WAYPOINT_Z 4



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

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    target_pose.pose = msg->pose.pose;
}

void velocityCallback(const geometry_msgs::TwistStamped &_velocity) {
    velocity = _velocity;
}

void ownPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    own_pose.pose = msg->pose;
}
void ownVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    own_velocity.twist = msg->twist;
}
bool directorEventCb(multidrone_msgs::DirectorEvent::Request &req,
                     multidrone_msgs::DirectorEvent::Request &res)
{
  event_received_id = req.event_id;
  event_received = true;
  ROS_INFO("event %s received from the Director", event_received_id.c_str());
  return true;
}


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

bool checkHovering(){
    Eigen::Vector3f current_pose, current_target_pose;
    current_pose = Eigen::Vector3f(own_pose.pose.position.x, own_pose.pose.position.y, own_pose.pose.position.z);
    current_target_pose = Eigen::Vector3f(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
    if((current_target_pose - current_pose).norm() > 0.5) return false;
    else return true;
}

std::vector<double> x;
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;
bool solver_called = false;



   



/** Control function
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
    while(event_received_id == "START_RACE" ){
        sleep(0.5);
    }
    /////////////////////////////////////////////////////////////
    while(ros::ok()){
        if(solver_called) cont = 0;
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
       /* pose.pose.position.x = x[cont];
        pose.pose.position.y = y[cont];
        pose.pose.position.z = z[cont];
        pose.pose.orientation = target_pose.pose.orientation;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();

        set_pose_pub.publish(pose); */
        /*
        
        /************* YAW *****************************/
        double dx = x[cont] - own_pose.pose.position.x;
        double dy = y[cont] - own_pose.pose.position.y;
        double yaw = atan2(dy,dx);
        geometry_msgs::Quaternion quat = toQuaternion(0.0, 0.0, yaw);

        pose.pose.orientation = quat;

        /************** VELOCITY ***********************/


        geometry_msgs::TwistStamped vel;
        vel.twist.linear.x = vx[10];
        vel.twist.linear.y = vy[10];
        vel.twist.linear.z = vz[10];

      
        vel.header.frame_id = "map";
        vel.header.stamp = ros::Time::now();
        set_velocity_pub.publish(vel);

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
    myparams.xinit[3] = 0;
    myparams.xinit[4] = 0;
    myparams.xinit[5] = 0;

    // set initial guess
    std::vector<double> x0;
    double x0i[] = {0, 0, 0, 0, 0, 15, 0.5, 0.5, 0.5};
    for (int j = 0; j < 30; j++)
    {
        for (i = 0; i < 9; i++)
        {
            x0.push_back(x0i[i]);
        }
    }

    for (i = 0; i < x0.size(); i++)
    {
        myparams.x0[i] = x0[i];
        // printing the params
        std::cout << myparams.x0[i] << std::endl;
    }
    // set parameters
    std::vector<double> params;
    double def_param[] = {WAYPOINT_X, WAYPOINT_Y, WAYPOINT_Z, 0, 0, 0};
    for(int i = 0; i<30; i++){
        for(int j=0; j<6; j++){
            params.push_back(def_param[j]);
        }
    }
    for(int i=0; i<params.size();i++) 
    {
        myparams.all_parameters[i] = params[i];
       // std::cout << params[i] << std::endl;

    }
     // call the solver
    
    exitflag = FORCESNLPsolver_solve(&myparams, &myoutput, &myinfo, stdout, pt2Function);
    solver_called = true;
    // save the output in a vector
    x.clear();
    y.clear();
    z.clear();         
    x.push_back(myoutput.x01[3]);
    x.push_back(myoutput.x02[3]);
    x.push_back(myoutput.x03[3]);
    x.push_back(myoutput.x04[3]);
    x.push_back(myoutput.x05[3]);
    x.push_back(myoutput.x06[3]);
    x.push_back(myoutput.x07[3]);
    x.push_back(myoutput.x08[3]);
    x.push_back(myoutput.x09[3]);
    x.push_back(myoutput.x10[3]);
    x.push_back(myoutput.x11[3]);
    x.push_back(myoutput.x12[3]);
    x.push_back(myoutput.x13[3]);
    x.push_back(myoutput.x14[3]);
    x.push_back(myoutput.x15[3]);
    x.push_back(myoutput.x16[3]);
    x.push_back(myoutput.x17[3]);
    x.push_back(myoutput.x18[3]);
    x.push_back(myoutput.x19[3]);
    x.push_back(myoutput.x20[3]);
    x.push_back(myoutput.x21[3]);
    x.push_back(myoutput.x22[3]);
    x.push_back(myoutput.x23[3]);
    x.push_back(myoutput.x24[3]);
    x.push_back(myoutput.x25[3]);
    x.push_back(myoutput.x26[3]);
    x.push_back(myoutput.x27[3]);
    x.push_back(myoutput.x28[3]);
    x.push_back(myoutput.x29[3]);
    x.push_back(myoutput.x30[3]);

    vx.push_back(myoutput.x01[6]);
    vx.push_back(myoutput.x02[6]);
    vx.push_back(myoutput.x03[6]);
    vx.push_back(myoutput.x04[6]);
    vx.push_back(myoutput.x05[6]);
    vx.push_back(myoutput.x06[6]);
    vx.push_back(myoutput.x07[6]);
    vx.push_back(myoutput.x08[6]);
    vx.push_back(myoutput.x09[6]);
    vx.push_back(myoutput.x10[6]);
    vx.push_back(myoutput.x11[6]);
    vx.push_back(myoutput.x12[6]);
    vx.push_back(myoutput.x13[6]);
    vx.push_back(myoutput.x14[6]);
    vx.push_back(myoutput.x15[6]);
    vx.push_back(myoutput.x16[6]);
    vx.push_back(myoutput.x17[6]);
    vx.push_back(myoutput.x18[6]);
    vx.push_back(myoutput.x19[6]);
    vx.push_back(myoutput.x20[6]);
    vx.push_back(myoutput.x21[6]);
    vx.push_back(myoutput.x22[6]);
    vx.push_back(myoutput.x23[6]);
    vx.push_back(myoutput.x24[6]);
    vx.push_back(myoutput.x25[6]);
    vx.push_back(myoutput.x26[6]);
    vx.push_back(myoutput.x27[6]);
    vx.push_back(myoutput.x28[6]);
    vx.push_back(myoutput.x29[6]);
    vx.push_back(myoutput.x30[6]);
    // xplot.push_back(myoutput->x31[3]);
    // xplot.push_back(myoutput->x32[3]);
    // xplot.push_back(myoutput->x33[3]);
    // xplot.push_back(myoutput->x34[3]);
    // xplot.push_back(myoutput->x35[3]);
    // xplot.push_back(myoutput->x36[3]);
    // xplot.push_back(myoutput->x37[3]);
    // xplot.push_back(myoutput->x38[3]);
    // xplot.push_back(myoutput->x39[3]);
    // xplot.push_back(myoutput->x40[3]);
    // xplot.push_back(myoutput->x41[3]);
    // xplot.push_back(myoutput->x42[3]);
    // xplot.push_back(myoutput->x43[3]);
    // xplot.push_back(myoutput->x44[3]);
    // xplot.push_back(myoutput->x45[3]);
    // xplot.push_back(myoutput->x46[3]);
    // xplot.push_back(myoutput->x47[3]);
    // xplot.push_back(myoutput->x48[3]);
    // xplot.push_back(myoutput->x49[3]);
    // xplot.push_back(myoutput->x50[3]);
    y.push_back(myoutput.x01[4]);
    y.push_back(myoutput.x02[4]);
    y.push_back(myoutput.x03[4]);
    y.push_back(myoutput.x04[4]);
    y.push_back(myoutput.x05[4]);
    y.push_back(myoutput.x06[4]);
    y.push_back(myoutput.x07[4]);
    y.push_back(myoutput.x08[4]);
    y.push_back(myoutput.x09[4]);
    y.push_back(myoutput.x10[4]);
    y.push_back(myoutput.x11[4]);
    y.push_back(myoutput.x12[4]);
    y.push_back(myoutput.x13[4]);
    y.push_back(myoutput.x14[4]);
    y.push_back(myoutput.x15[4]);
    y.push_back(myoutput.x16[4]);
    y.push_back(myoutput.x17[4]);
    y.push_back(myoutput.x18[4]);
    y.push_back(myoutput.x19[4]);
    y.push_back(myoutput.x20[4]);
    y.push_back(myoutput.x21[4]);
    y.push_back(myoutput.x22[4]);
    y.push_back(myoutput.x23[4]);
    y.push_back(myoutput.x24[4]);
    y.push_back(myoutput.x25[4]);
    y.push_back(myoutput.x26[4]);
    y.push_back(myoutput.x27[4]);
    y.push_back(myoutput.x28[4]);
    y.push_back(myoutput.x29[4]);
    y.push_back(myoutput.x30[4]);


    vy.push_back(myoutput.x01[7]);
    vy.push_back(myoutput.x02[7]);
    vy.push_back(myoutput.x03[7]);
    vy.push_back(myoutput.x04[7]);
    vy.push_back(myoutput.x05[7]);
    vy.push_back(myoutput.x06[7]);
    vy.push_back(myoutput.x07[7]);
    vy.push_back(myoutput.x08[7]);
    vy.push_back(myoutput.x09[7]);
    vy.push_back(myoutput.x10[7]);
    vy.push_back(myoutput.x11[7]);
    vy.push_back(myoutput.x12[7]);
    vy.push_back(myoutput.x13[7]);
    vy.push_back(myoutput.x14[7]);
    vy.push_back(myoutput.x15[7]);
    vy.push_back(myoutput.x16[7]);
    vy.push_back(myoutput.x17[7]);
    vy.push_back(myoutput.x18[7]);
    vy.push_back(myoutput.x19[7]);
    vy.push_back(myoutput.x20[7]);
    vy.push_back(myoutput.x21[7]);
    vy.push_back(myoutput.x22[7]);
    vy.push_back(myoutput.x23[7]);
    vy.push_back(myoutput.x24[7]);
    vy.push_back(myoutput.x25[7]);
    vy.push_back(myoutput.x26[7]);
    vy.push_back(myoutput.x27[7]);
    vy.push_back(myoutput.x28[7]);
    vy.push_back(myoutput.x29[7]);
    vy.push_back(myoutput.x30[7]);
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

    vz.push_back(myoutput.x01[8]);
    vz.push_back(myoutput.x02[8]);
    vz.push_back(myoutput.x03[8]);
    vz.push_back(myoutput.x04[8]);
    vz.push_back(myoutput.x05[8]);
    vz.push_back(myoutput.x06[8]);
    vz.push_back(myoutput.x07[8]);
    vz.push_back(myoutput.x08[8]);
    vz.push_back(myoutput.x09[8]);
    vz.push_back(myoutput.x10[8]);
    vz.push_back(myoutput.x11[8]);
    vz.push_back(myoutput.x12[8]);
    vz.push_back(myoutput.x13[8]);
    vz.push_back(myoutput.x14[8]);
    vz.push_back(myoutput.x15[8]);
    vz.push_back(myoutput.x16[8]);
    vz.push_back(myoutput.x17[8]);
    vz.push_back(myoutput.x18[8]);
    vz.push_back(myoutput.x19[8]);
    vz.push_back(myoutput.x20[8]);
    vz.push_back(myoutput.x21[8]);
    vz.push_back(myoutput.x22[8]);
    vz.push_back(myoutput.x23[8]);
    vz.push_back(myoutput.x24[8]);
    vz.push_back(myoutput.x25[8]);
    vz.push_back(myoutput.x26[8]);
    vz.push_back(myoutput.x27[8]);
    vz.push_back(myoutput.x28[8]);
    vz.push_back(myoutput.x29[8]);
    vz.push_back(myoutput.x30[8]);

    z.push_back(myoutput.x01[5]);
    z.push_back(myoutput.x02[5]);
    z.push_back(myoutput.x03[5]);
    z.push_back(myoutput.x04[5]);
    z.push_back(myoutput.x05[5]);
    z.push_back(myoutput.x06[5]);
    z.push_back(myoutput.x07[5]);
    z.push_back(myoutput.x08[5]);
    z.push_back(myoutput.x09[5]);
    z.push_back(myoutput.x10[5]);
    z.push_back(myoutput.x11[5]);
    z.push_back(myoutput.x12[5]);
    z.push_back(myoutput.x13[5]);
    z.push_back(myoutput.x14[5]);
    z.push_back(myoutput.x15[5]);
    z.push_back(myoutput.x16[5]);
    z.push_back(myoutput.x17[5]);
    z.push_back(myoutput.x18[5]);
    z.push_back(myoutput.x19[5]);
    z.push_back(myoutput.x20[5]);
    z.push_back(myoutput.x21[5]);
    z.push_back(myoutput.x22[5]);
    z.push_back(myoutput.x23[5]);
    z.push_back(myoutput.x24[5]);
    z.push_back(myoutput.x25[5]);
    z.push_back(myoutput.x26[5]);
    z.push_back(myoutput.x27[5]);
    z.push_back(myoutput.x28[5]);
    z.push_back(myoutput.x29[5]);
    z.push_back(myoutput.x30[5]);
    // publish a trajectory topic
    //std::cout<<"x: "<<x[30]<< "y: "<<y[30]<<std::endl;

    //

    if (exitflag == 1)
    {
      // print the exitflag for RVIZ    
    }
    else
    {
        //std::cout << "exitflag: " << exitflag << std::endl;
    }
    
}

/**  Hector function
 */

nav_msgs::Path constructPath(std::vector<double> wps_x, std::vector<double> wps_y, std::vector<double> wps_z) {
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
    sub_velocity = nh.subscribe("/uav_path_manager/follower/uav_1/output_vel", 0, &velocityCallback);
    ros::ServiceServer service = nh.advertiseService("event_manager/director_event", directorEventCb);



    // thread for calling the UAL
    std::thread threadObjSolver(UALthread);


    ros::Rate rate(1);//hz

    ROS_INFO("solver running");
    while(ros::ok){

        // solver function
        solverFunction();
        /***************** HECTOR ************************ /


        /*nav_msgs::Path init_path = constructPath(x,y,z);
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

       // path_rviz_pub.publish(path);
        rate.sleep();
        ros::spinOnce();
    }    
    //threadObjSolver.join();
    return 0;
}





/*void plottingResult(FORCESNLPsolver_output *myoutput){

    plt::plot(x, y, "b");
    //plt::plot(xcircle, ycircle, "r");
    plt::xlim(-6.0, 11.0);
    plt::ylim(-6.0, 6.0);
    plt::show();
    // plotting the result

}
*/

