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

geometry_msgs::PoseStamped target_pose;
geometry_msgs::PoseStamped own_pose;



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

void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    target_pose.pose = msg->pose;
    //std::cout<<"x: "<<target_pose.pose.position.x<<" y: "<<target_pose.pose.position.y<<" z "<<target_pose.pose.position.z<<std::endl;
}

void ownPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    own_pose.pose = msg->pose;
}

std::vector<double> x;
std::vector<double> y;
std::vector<double> z;
bool solver_called = false;

void UALthread(){
    int cont = 0;
    ros::Rate rate(10);//hz
    sleep(10);
    /////////////////////////////////////////////////////////////
    while(ros::ok()){
        if(solver_called) cont = 0;
        // call the UAL
        uav_abstraction_layer::GoToWaypoint pose_srv;
        pose_srv.request.waypoint.pose.position.x = x[cont];
        pose_srv.request.waypoint.pose.position.y = y[cont];
        pose_srv.request.waypoint.pose.position.z = z[cont];
        pose_srv.request.waypoint.header.frame_id = "map";
        pose_srv.request.waypoint.header.stamp = ros::Time::now();
        pose_srv.request.blocking = false;

        cont = cont+1;
        ros::spinOnce();
        rate.sleep();
    }
}



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
    double def_param[] = {target_pose.pose.position.x,target_pose.pose.position.y, target_pose.pose.position.z, 0, 0, 0,15,15};
    for(int i = 0; i<30; i++){
        for(int j=0; j<8; j++){
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

    ros::Subscriber target_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav_1/ual/pose", 1, targetPoseCallback);
    ros::Subscriber own_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("uav_2/ual/pose", 1, ownPoseCallback);
    ros::ServiceClient go_to_waypoint_srv_ = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("uav_2/ual/go_to_waypoint");
    ros::ServiceClient take_off_srv_ = nh.serviceClient<uav_abstraction_layer::TakeOff>("uav_2/ual/take_off");

    ros::Publisher path_rviz_pub = nh.advertise<nav_msgs::Path>("/solver/path",1);

    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = 5;
    take_off_srv_.call(srv);
    sleep(6);
    // thread for calling the UAL
   // std::thread threadObjSolver(UALthread);

    ROS_INFO("aqui");

 

    ros::Rate rate(1);//hz

    while(ros::ok){
        // solver function
        solverFunction();
        //plotting RVIZ
        nav_msgs::Path path_to_publish = constructPath(x,y,z);
        path_rviz_pub.publish(path_to_publish);
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

