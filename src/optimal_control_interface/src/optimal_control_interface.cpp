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
    csv_ual << msg->pose.position.x << ", " << msg->pose.position.y << ", " << msg->pose.position.z << std::endl;
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
    csv_ual.open("/home/grvc/ual.csv");

}
///////////////// UTILITY FUNCTIONS ////////////////////

/**
 */

void calculateDesiredPoint(int shooting_type, const std::vector<double> &target_pose, std::vector<double> &desired_pose, std::vector<double> desired_velocity){
    switch(shooting_type){
        //TODO
        case orbital:
            desired_pose[0] = target_pose[0];
            desired_pose[1] = target_pose[1];
            desired_pose[2] = target_pose[2];
        break;
        case lateral:
            desired_pose[0] = target_pose[0];
            desired_pose[1] = target_pose[1];
            desired_pose[2] = target_pose[2];
        break;
        case flyover:
            desired_pose[0] = target_pose[0];
            desired_pose[1] = target_pose[1];
            desired_pose[2] = target_pose[2];
        break;
    } 
}

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
        csv_pose << x[i] << ", " << y[i] << ", " << z[i] << std::endl;
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

void publishPath(std::vector<double> &wps_x, std::vector<double> &wps_y, std::vector<double> &wps_z, std::vector<double> &desired_wp, std::vector<double> &target_pose_init, std::vector<double> &target_vel ) {
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
    double x0i[] = {u_x, u_y, u_z, p_x, p_y, p_z, v_x, v_y, v_z, t_x, t_y};
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
                        obst[0], obst[1],target_pose_init[0],target_pose_init[1],target_vel[0],target_vel[1]};

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
    x.push_back(myoutput.x001[position_x]);
    x.push_back(myoutput.x002[position_x]);
    x.push_back(myoutput.x003[position_x]);
    x.push_back(myoutput.x004[position_x]);
    x.push_back(myoutput.x005[position_x]);
    x.push_back(myoutput.x006[position_x]);
    x.push_back(myoutput.x007[position_x]);
    x.push_back(myoutput.x008[position_x]);
    x.push_back(myoutput.x009[position_x]);
    x.push_back(myoutput.x010[position_x]);
    x.push_back(myoutput.x011[position_x]);
    x.push_back(myoutput.x012[position_x]);
    x.push_back(myoutput.x013[position_x]);
    x.push_back(myoutput.x014[position_x]);
    x.push_back(myoutput.x015[position_x]);
    x.push_back(myoutput.x016[position_x]);
    x.push_back(myoutput.x017[position_x]);
    x.push_back(myoutput.x018[position_x]);
    x.push_back(myoutput.x019[position_x]);
    x.push_back(myoutput.x020[position_x]);
    x.push_back(myoutput.x021[position_x]);
    x.push_back(myoutput.x022[position_x]);
    x.push_back(myoutput.x023[position_x]);
    x.push_back(myoutput.x024[position_x]);
    x.push_back(myoutput.x025[position_x]);
    x.push_back(myoutput.x026[position_x]);
    x.push_back(myoutput.x027[position_x]);
    x.push_back(myoutput.x028[position_x]);
    x.push_back(myoutput.x029[position_x]);
    x.push_back(myoutput.x030[position_x]);
    x.push_back(myoutput.x031[position_x]);
    x.push_back(myoutput.x032[position_x]);
    x.push_back(myoutput.x033[position_x]);
    x.push_back(myoutput.x034[position_x]);
    x.push_back(myoutput.x035[position_x]);
    x.push_back(myoutput.x036[position_x]);
    x.push_back(myoutput.x037[position_x]);
    x.push_back(myoutput.x038[position_x]);
    x.push_back(myoutput.x039[position_x]);
    x.push_back(myoutput.x040[position_x]);
    x.push_back(myoutput.x041[position_x]);
    x.push_back(myoutput.x042[position_x]);
    x.push_back(myoutput.x043[position_x]);
    x.push_back(myoutput.x044[position_x]);
    x.push_back(myoutput.x045[position_x]);
    x.push_back(myoutput.x046[position_x]);
    x.push_back(myoutput.x047[position_x]);
    x.push_back(myoutput.x048[position_x]);
    x.push_back(myoutput.x049[position_x]);
    x.push_back(myoutput.x050[position_x]);
    x.push_back(myoutput.x051[position_x]);
    x.push_back(myoutput.x052[position_x]);
    x.push_back(myoutput.x053[position_x]);
    x.push_back(myoutput.x054[position_x]);
    x.push_back(myoutput.x055[position_x]);
    x.push_back(myoutput.x056[position_x]);
    x.push_back(myoutput.x057[position_x]);
    x.push_back(myoutput.x058[position_x]);
    x.push_back(myoutput.x059[position_x]);
    x.push_back(myoutput.x060[position_x]);
    x.push_back(myoutput.x061[position_x]);
    x.push_back(myoutput.x062[position_x]);
    x.push_back(myoutput.x063[position_x]);
    x.push_back(myoutput.x064[position_x]);
    x.push_back(myoutput.x065[position_x]);
    x.push_back(myoutput.x066[position_x]);
    x.push_back(myoutput.x067[position_x]);
    x.push_back(myoutput.x068[position_x]);
    x.push_back(myoutput.x069[position_x]);
    x.push_back(myoutput.x070[position_x]);
    x.push_back(myoutput.x071[position_x]);
    x.push_back(myoutput.x072[position_x]);
    x.push_back(myoutput.x073[position_x]);
    x.push_back(myoutput.x074[position_x]);
    x.push_back(myoutput.x075[position_x]);
    x.push_back(myoutput.x076[position_x]);
    x.push_back(myoutput.x077[position_x]);
    x.push_back(myoutput.x078[position_x]);
    x.push_back(myoutput.x079[position_x]);
    x.push_back(myoutput.x080[position_x]);
    x.push_back(myoutput.x081[position_x]);
    x.push_back(myoutput.x082[position_x]);
    x.push_back(myoutput.x083[position_x]);
    x.push_back(myoutput.x084[position_x]);
    x.push_back(myoutput.x085[position_x]);
    x.push_back(myoutput.x086[position_x]);
    x.push_back(myoutput.x087[position_x]);
    x.push_back(myoutput.x088[position_x]);
    x.push_back(myoutput.x089[position_x]);
    x.push_back(myoutput.x090[position_x]);
    x.push_back(myoutput.x091[position_x]);
    x.push_back(myoutput.x092[position_x]);
    x.push_back(myoutput.x093[position_x]);
    x.push_back(myoutput.x094[position_x]);
    x.push_back(myoutput.x095[position_x]);
    x.push_back(myoutput.x096[position_x]);
    x.push_back(myoutput.x097[position_x]);
    x.push_back(myoutput.x098[position_x]);
    x.push_back(myoutput.x099[position_x]);
    x.push_back(myoutput.x100[position_x]);



    vx.push_back(myoutput.x001[velocity_x]);
    vx.push_back(myoutput.x002[velocity_x]);
    vx.push_back(myoutput.x003[velocity_x]);
    vx.push_back(myoutput.x004[velocity_x]);
    vx.push_back(myoutput.x005[velocity_x]);
    vx.push_back(myoutput.x006[velocity_x]);
    vx.push_back(myoutput.x007[velocity_x]);
    vx.push_back(myoutput.x008[velocity_x]);
    vx.push_back(myoutput.x009[velocity_x]);
    vx.push_back(myoutput.x010[velocity_x]);
    vx.push_back(myoutput.x011[velocity_x]);
    vx.push_back(myoutput.x012[velocity_x]);
    vx.push_back(myoutput.x013[velocity_x]);
    vx.push_back(myoutput.x014[velocity_x]);
    vx.push_back(myoutput.x015[velocity_x]);
    vx.push_back(myoutput.x016[velocity_x]);
    vx.push_back(myoutput.x017[velocity_x]);
    vx.push_back(myoutput.x018[velocity_x]);
    vx.push_back(myoutput.x019[velocity_x]);
    vx.push_back(myoutput.x020[velocity_x]);
    vx.push_back(myoutput.x021[velocity_x]);
    vx.push_back(myoutput.x022[velocity_x]);
    vx.push_back(myoutput.x023[velocity_x]);
    vx.push_back(myoutput.x024[velocity_x]);
    vx.push_back(myoutput.x025[velocity_x]);
    vx.push_back(myoutput.x026[velocity_x]);
    vx.push_back(myoutput.x027[velocity_x]);
    vx.push_back(myoutput.x028[velocity_x]);
    vx.push_back(myoutput.x029[velocity_x]);
    vx.push_back(myoutput.x030[velocity_x]);
    vx.push_back(myoutput.x031[velocity_x]);
    vx.push_back(myoutput.x032[velocity_x]);
    vx.push_back(myoutput.x033[velocity_x]);
    vx.push_back(myoutput.x034[velocity_x]);
    vx.push_back(myoutput.x035[velocity_x]);
    vx.push_back(myoutput.x036[velocity_x]);
    vx.push_back(myoutput.x037[velocity_x]);
    vx.push_back(myoutput.x038[velocity_x]);
    vx.push_back(myoutput.x039[velocity_x]);
    vx.push_back(myoutput.x040[velocity_x]);
    vx.push_back(myoutput.x041[velocity_x]);
    vx.push_back(myoutput.x042[velocity_x]);
    vx.push_back(myoutput.x043[velocity_x]);
    vx.push_back(myoutput.x044[velocity_x]);
    vx.push_back(myoutput.x045[velocity_x]);
    vx.push_back(myoutput.x046[velocity_x]);
    vx.push_back(myoutput.x047[velocity_x]);
    vx.push_back(myoutput.x048[velocity_x]);
    vx.push_back(myoutput.x049[velocity_x]);
    vx.push_back(myoutput.x050[velocity_x]);
   

    y.push_back(myoutput.x001[position_y]);
    y.push_back(myoutput.x002[position_y]);
    y.push_back(myoutput.x003[position_y]);
    y.push_back(myoutput.x004[position_y]);
    y.push_back(myoutput.x005[position_y]);
    y.push_back(myoutput.x006[position_y]);
    y.push_back(myoutput.x007[position_y]);
    y.push_back(myoutput.x008[position_y]);
    y.push_back(myoutput.x009[position_y]);
    y.push_back(myoutput.x010[position_y]);
    y.push_back(myoutput.x011[position_y]);
    y.push_back(myoutput.x012[position_y]);
    y.push_back(myoutput.x013[position_y]);
    y.push_back(myoutput.x014[position_y]);
    y.push_back(myoutput.x015[position_y]);
    y.push_back(myoutput.x016[position_y]);
    y.push_back(myoutput.x017[position_y]);
    y.push_back(myoutput.x018[position_y]);
    y.push_back(myoutput.x019[position_y]);
    y.push_back(myoutput.x020[position_y]);
    y.push_back(myoutput.x021[position_y]);
    y.push_back(myoutput.x022[position_y]);
    y.push_back(myoutput.x023[position_y]);
    y.push_back(myoutput.x024[position_y]);
    y.push_back(myoutput.x025[position_y]);
    y.push_back(myoutput.x026[position_y]);
    y.push_back(myoutput.x027[position_y]);
    y.push_back(myoutput.x028[position_y]);
    y.push_back(myoutput.x029[position_y]);
    y.push_back(myoutput.x030[position_y]);
    y.push_back(myoutput.x031[position_y]);
    y.push_back(myoutput.x032[position_y]);
    y.push_back(myoutput.x033[position_y]);
    y.push_back(myoutput.x034[position_y]);
    y.push_back(myoutput.x035[position_y]);
    y.push_back(myoutput.x036[position_y]);
    y.push_back(myoutput.x037[position_y]);
    y.push_back(myoutput.x038[position_y]);
    y.push_back(myoutput.x039[position_y]);
    y.push_back(myoutput.x040[position_y]);
    y.push_back(myoutput.x041[position_y]);
    y.push_back(myoutput.x042[position_y]);
    y.push_back(myoutput.x043[position_y]);
    y.push_back(myoutput.x044[position_y]);
    y.push_back(myoutput.x045[position_y]);
    y.push_back(myoutput.x046[position_y]);
    y.push_back(myoutput.x047[position_y]);
    y.push_back(myoutput.x048[position_y]);
    y.push_back(myoutput.x049[position_y]);
    y.push_back(myoutput.x050[position_y]);
    y.push_back(myoutput.x051[position_y]);
    y.push_back(myoutput.x052[position_y]);
    y.push_back(myoutput.x053[position_y]);
    y.push_back(myoutput.x054[position_y]);
    y.push_back(myoutput.x055[position_y]);
    y.push_back(myoutput.x056[position_y]);
    y.push_back(myoutput.x057[position_y]);
    y.push_back(myoutput.x058[position_y]);
    y.push_back(myoutput.x059[position_y]);
    y.push_back(myoutput.x060[position_y]);
    y.push_back(myoutput.x061[position_y]);
    y.push_back(myoutput.x062[position_y]);
    y.push_back(myoutput.x063[position_y]);
    y.push_back(myoutput.x064[position_y]);
    y.push_back(myoutput.x065[position_y]);
    y.push_back(myoutput.x066[position_y]);
    y.push_back(myoutput.x067[position_y]);
    y.push_back(myoutput.x068[position_y]);
    y.push_back(myoutput.x069[position_y]);
    y.push_back(myoutput.x070[position_y]);
    y.push_back(myoutput.x071[position_y]);
    y.push_back(myoutput.x072[position_y]);
    y.push_back(myoutput.x073[position_y]);
    y.push_back(myoutput.x074[position_y]);
    y.push_back(myoutput.x075[position_y]);
    y.push_back(myoutput.x076[position_y]);
    y.push_back(myoutput.x077[position_y]);
    y.push_back(myoutput.x078[position_y]);
    y.push_back(myoutput.x079[position_y]);
    y.push_back(myoutput.x080[position_y]);
    y.push_back(myoutput.x081[position_y]);
    y.push_back(myoutput.x082[position_y]);
    y.push_back(myoutput.x083[position_y]);
    y.push_back(myoutput.x084[position_y]);
    y.push_back(myoutput.x085[position_y]);
    y.push_back(myoutput.x086[position_y]);
    y.push_back(myoutput.x087[position_y]);
    y.push_back(myoutput.x088[position_y]);
    y.push_back(myoutput.x089[position_y]);
    y.push_back(myoutput.x090[position_y]);
    y.push_back(myoutput.x091[position_y]);
    y.push_back(myoutput.x092[position_y]);
    y.push_back(myoutput.x093[position_y]);
    y.push_back(myoutput.x094[position_y]);
    y.push_back(myoutput.x095[position_y]);
    y.push_back(myoutput.x096[position_y]);
    y.push_back(myoutput.x097[position_y]);
    y.push_back(myoutput.x098[position_y]);
    y.push_back(myoutput.x099[position_y]);
    y.push_back(myoutput.x100[position_y]);

    vy.push_back(myoutput.x001[velocity_y]);
    vy.push_back(myoutput.x002[velocity_y]);
    vy.push_back(myoutput.x003[velocity_y]);
    vy.push_back(myoutput.x004[velocity_y]);
    vy.push_back(myoutput.x005[velocity_y]);
    vy.push_back(myoutput.x006[velocity_y]);
    vy.push_back(myoutput.x007[velocity_y]);
    vy.push_back(myoutput.x008[velocity_y]);
    vy.push_back(myoutput.x009[velocity_y]);
    vy.push_back(myoutput.x010[velocity_y]);
    vy.push_back(myoutput.x011[velocity_y]);
    vy.push_back(myoutput.x012[velocity_y]);
    vy.push_back(myoutput.x013[velocity_y]);
    vy.push_back(myoutput.x014[velocity_y]);
    vy.push_back(myoutput.x015[velocity_y]);
    vy.push_back(myoutput.x016[velocity_y]);
    vy.push_back(myoutput.x017[velocity_y]);
    vy.push_back(myoutput.x018[velocity_y]);
    vy.push_back(myoutput.x019[velocity_y]);
    vy.push_back(myoutput.x020[velocity_y]);
    vy.push_back(myoutput.x021[velocity_y]);
    vy.push_back(myoutput.x022[velocity_y]);
    vy.push_back(myoutput.x023[velocity_y]);
    vy.push_back(myoutput.x024[velocity_y]);
    vy.push_back(myoutput.x025[velocity_y]);
    vy.push_back(myoutput.x026[velocity_y]);
    vy.push_back(myoutput.x027[velocity_y]);
    vy.push_back(myoutput.x028[velocity_y]);
    vy.push_back(myoutput.x029[velocity_y]);
    vy.push_back(myoutput.x030[velocity_y]);
    vy.push_back(myoutput.x031[velocity_y]);
    vy.push_back(myoutput.x032[velocity_y]);
    vy.push_back(myoutput.x033[velocity_y]);
    vy.push_back(myoutput.x034[velocity_y]);
    vy.push_back(myoutput.x035[velocity_y]);
    vy.push_back(myoutput.x036[velocity_y]);
    vy.push_back(myoutput.x037[velocity_y]);
    vy.push_back(myoutput.x038[velocity_y]);
    vy.push_back(myoutput.x039[velocity_y]);
    vy.push_back(myoutput.x040[velocity_y]);
    vy.push_back(myoutput.x041[velocity_y]);
    vy.push_back(myoutput.x042[velocity_y]);
    vy.push_back(myoutput.x043[velocity_y]);
    vy.push_back(myoutput.x044[velocity_y]);
    vy.push_back(myoutput.x045[velocity_y]);
    vy.push_back(myoutput.x046[velocity_y]);
    vy.push_back(myoutput.x047[velocity_y]);
    vy.push_back(myoutput.x048[velocity_y]);
    vy.push_back(myoutput.x049[velocity_y]);
    vy.push_back(myoutput.x050[velocity_y]);


    vz.push_back(myoutput.x001[velocity_z]);
    vz.push_back(myoutput.x002[velocity_z]);
    vz.push_back(myoutput.x003[velocity_z]);
    vz.push_back(myoutput.x004[velocity_z]);
    vz.push_back(myoutput.x005[velocity_z]);
    vz.push_back(myoutput.x006[velocity_z]);
    vz.push_back(myoutput.x007[velocity_z]);
    vz.push_back(myoutput.x008[velocity_z]);
    vz.push_back(myoutput.x009[velocity_z]);
    vz.push_back(myoutput.x010[velocity_z]);
    vz.push_back(myoutput.x011[velocity_z]);
    vz.push_back(myoutput.x012[velocity_z]);
    vz.push_back(myoutput.x013[velocity_z]);
    vz.push_back(myoutput.x014[velocity_z]);
    vz.push_back(myoutput.x015[velocity_z]);
    vz.push_back(myoutput.x016[velocity_z]);
    vz.push_back(myoutput.x017[velocity_z]);
    vz.push_back(myoutput.x018[velocity_z]);
    vz.push_back(myoutput.x019[velocity_z]);
    vz.push_back(myoutput.x020[velocity_z]);
    vz.push_back(myoutput.x021[velocity_z]);
    vz.push_back(myoutput.x022[velocity_z]);
    vz.push_back(myoutput.x023[velocity_z]);
    vz.push_back(myoutput.x024[velocity_z]);
    vz.push_back(myoutput.x025[velocity_z]);
    vz.push_back(myoutput.x026[velocity_z]);
    vz.push_back(myoutput.x027[velocity_z]);
    vz.push_back(myoutput.x028[velocity_z]);
    vz.push_back(myoutput.x029[velocity_z]);
    vz.push_back(myoutput.x030[velocity_z]);
    vz.push_back(myoutput.x031[velocity_z]);
    vz.push_back(myoutput.x032[velocity_z]);
    vz.push_back(myoutput.x033[velocity_z]);
    vz.push_back(myoutput.x034[velocity_z]);
    vz.push_back(myoutput.x035[velocity_z]);
    vz.push_back(myoutput.x036[velocity_z]);
    vz.push_back(myoutput.x037[velocity_z]);
    vz.push_back(myoutput.x038[velocity_z]);
    vz.push_back(myoutput.x039[velocity_z]);
    vz.push_back(myoutput.x040[velocity_z]);
    vz.push_back(myoutput.x041[velocity_z]);
    vz.push_back(myoutput.x042[velocity_z]);
    vz.push_back(myoutput.x043[velocity_z]);
    vz.push_back(myoutput.x044[velocity_z]);
    vz.push_back(myoutput.x045[velocity_z]);
    vz.push_back(myoutput.x046[velocity_z]);
    vz.push_back(myoutput.x047[velocity_z]);
    vz.push_back(myoutput.x048[velocity_z]);
    vz.push_back(myoutput.x049[velocity_z]);
    vz.push_back(myoutput.x050[velocity_z]);


    z.push_back(myoutput.x001[position_z]);
    z.push_back(myoutput.x002[position_z]);
    z.push_back(myoutput.x003[position_z]);
    z.push_back(myoutput.x004[position_z]);
    z.push_back(myoutput.x005[position_z]);
    z.push_back(myoutput.x006[position_z]);
    z.push_back(myoutput.x007[position_z]);
    z.push_back(myoutput.x008[position_z]);
    z.push_back(myoutput.x009[position_z]);
    z.push_back(myoutput.x010[position_z]);
    z.push_back(myoutput.x011[position_z]);
    z.push_back(myoutput.x012[position_z]);
    z.push_back(myoutput.x013[position_z]);
    z.push_back(myoutput.x014[position_z]);
    z.push_back(myoutput.x015[position_z]);
    z.push_back(myoutput.x016[position_z]);
    z.push_back(myoutput.x017[position_z]);
    z.push_back(myoutput.x018[position_z]);
    z.push_back(myoutput.x019[position_z]);
    z.push_back(myoutput.x020[position_z]);
    z.push_back(myoutput.x021[position_z]);
    z.push_back(myoutput.x022[position_z]);
    z.push_back(myoutput.x023[position_z]);
    z.push_back(myoutput.x024[position_z]);
    z.push_back(myoutput.x025[position_z]);
    z.push_back(myoutput.x026[position_z]);
    z.push_back(myoutput.x027[position_z]);
    z.push_back(myoutput.x028[position_z]);
    z.push_back(myoutput.x029[position_z]);
    z.push_back(myoutput.x030[position_z]);
    z.push_back(myoutput.x031[position_z]);
    z.push_back(myoutput.x032[position_z]);
    z.push_back(myoutput.x033[position_z]);
    z.push_back(myoutput.x034[position_z]);
    z.push_back(myoutput.x035[position_z]);
    z.push_back(myoutput.x036[position_z]);
    z.push_back(myoutput.x037[position_z]);
    z.push_back(myoutput.x038[position_z]);
    z.push_back(myoutput.x039[position_z]);
    z.push_back(myoutput.x040[position_z]);
    z.push_back(myoutput.x041[position_z]);
    z.push_back(myoutput.x042[position_z]);
    z.push_back(myoutput.x043[position_z]);
    z.push_back(myoutput.x044[position_z]);
    z.push_back(myoutput.x045[position_z]);
    z.push_back(myoutput.x046[position_z]);
    z.push_back(myoutput.x047[position_z]);
    z.push_back(myoutput.x048[position_z]);
    z.push_back(myoutput.x049[position_z]);
    z.push_back(myoutput.x050[position_z]);
    
    if (exitflag == 1) return true;
    else return false;
    
}







