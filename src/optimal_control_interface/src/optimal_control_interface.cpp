/** ros node to interface with solver created by FORCES PRO */
#include <optimal_control_interface.h>


int offset = 10;

#ifdef __cplusplus
extern "C"
{
#endif
    extern void FORCESNLPsolver_casadi2forces(double *x, double *y, double *l, double *p,
                          double *f, double *nabla_f, double *c, double *nabla_c,
                          double *h, double *nabla_h, double *H, int stage, int iteration);
#ifdef __cplusplus
}
#endif
namespace plt = matplotlibcpp;

/////////////////////////////////// CALLBACKS //////////////////////////////////////////////



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


void init(ros::NodeHandle nh){
    sub_position = nh.subscribe<geometry_msgs::PoseStamped>("/drone_1/ual/pose",1,ownPoseCallback);
    sub_velocity = nh.subscribe<geometry_msgs::TwistStamped>("/drone_1/ual/velocity",1,ownVelocityCallback);
    path_rviz_pub = nh.advertise<nav_msgs::Path>("/solver/path",1);
    path_no_fly_zone = nh.advertise<nav_msgs::Path>("/solver/noflyzone",1);
    csv_pose.open("/home/grvc/pose.csv");
    csv_pose << std::fixed << std::setprecision(5);

}
///////////////// UTILITY FUNCTIONS ////////////////////


/** Utility function for hovering
 */

bool checkHovering(bool control_position){
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
  /*
  event_received_id = req.event_id;
  event_received = true;
  ROS_INFO("event %s received from the Director", event_received_id.c_str());
  return true;
  */
}

/** Utility function for plotting the result through matplotlib
 */

void plottingResult(FORCESNLPsolver_output *myoutput){

    //plt::plot(x, y, "b");
    plt::xlim(6.0, 11.0);
    plt::ylim(6.0, 6.0);
    plt::show();

}

/** function for logging to csv file
 */
void logToCsv(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz, int n_steps){
    // logging all results
     for(int i=0; i<n_steps; i++){
        csv_pose << own_pose.pose.position.x << ", " << own_pose.pose.position.y << ", " << own_pose.pose.position.z << std::endl;
    }
}
/** Construct the no fly zone path to visualize it on RVIZ */
void publishNoFlyZone(double point_1[2], double point_2[2],double point_3[2], double point_4[2]){
    nav_msgs::Path msg;
    msg.header.frame_id = "map";

    std::vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose;

    pose.pose.position.x = point_1[0];
    pose.pose.position.y = point_1[1];
    poses.push_back(pose);

    pose.pose.position.x = point_2[0];
    pose.pose.position.y = point_2[1];
    poses.push_back(pose);

    pose.pose.position.x = point_3[0];
    pose.pose.position.y = point_3[1];
    poses.push_back(pose);

    pose.pose.position.x = point_4[0];
    pose.pose.position.y = point_4[1];
    poses.push_back(pose);

    pose.pose.position.x = point_1[0];
    pose.pose.position.y = point_1[1];

    
    poses.push_back(pose);

    msg.poses = poses;
    path_no_fly_zone.publish(msg);
}

/**
 */
void publishDesiredPoint(double x, double y,double z){
    geometry_msgs::PointStamped desired_point;
    desired_point.point.x = x;
    desired_point.point.y = y;
    desired_point.point.z = z;
    desired_point.header.frame_id = "map";

    desired_pose_publisher.publish(desired_point);
}

/**  Construct a nav_msgs_path
 */

void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z, std::vector<double> &desired_wp) {
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
    path_rviz_pub.publish(msg);
}


//////////////////////////////////////////////////////////////

/** Control function. This function interfaces the MPC with UAL
 */

void UALthread(){
   
}

/////////////////////////////////////////////////////////////////////////////


/** solver function
*/

bool solverFunction(std::vector<double> &x, std::vector<double> &y, std::vector<double> &z, std::vector<double> &vx, std::vector<double> &vy, std::vector<double> &vz,std::vector<double> &desired_wp, std::vector<double> desired_vel, std::vector<double> &obst){

    /* declare FORCES variables and structures */
    FORCESNLPsolver_info myinfo;
    FORCESNLPsolver_params myparams;
    FORCESNLPsolver_output myoutput;
   
    /* define external function evaluating functions and derivatives (only for the high-level interface) */

    FORCESNLPsolver_extfunc pt2Function = &FORCESNLPsolver_casadi2forces;

    int i, exitflag;

    // set initial postion and velocity
    initial_time = clock(); 
    myparams.xinit[0] = own_pose.pose.position.x;
    myparams.xinit[1] = own_pose.pose.position.y;
    myparams.xinit[2] = own_pose.pose.position.z;
    myparams.xinit[3] = own_velocity.twist.linear.x;
    myparams.xinit[4] = own_velocity.twist.linear.y;
    myparams.xinit[5] = own_velocity.twist.linear.z; 

    // set initial guess
    std::vector<double> x0;
    double x0i[] = {u_x, u_y, u_z, p_x, p_y, p_z, v_x, v_y, v_z};
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
    double def_param[] = {desired_wp[0], desired_wp[1], desired_wp[2],
                        desired_vel[0], desired_vel[1], desired_vel[2],
                        obst[0], obst[1]};

    for(int i = 0; i<time_horizon; i++){
        for(int j=0; j<sizeof(def_param)/sizeof(double); j++){
            params.push_back(def_param[j]);
        }
    }
    for(int i=0; i<params.size();i++) 
    {
        myparams.all_parameters[i] = params[i];
    }

    // call the solver
    exitflag = FORCESNLPsolver_solve(&myparams, &myoutput, &myinfo, stdout, pt2Function);
    // save the output in a vector
  

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
    
    
    if (exitflag == 1) return true;
    else return false;
    
}







