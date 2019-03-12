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
#include <uav_abstraction_layer/State.h>

bool position_control;
ros::Publisher set_pose_pub;
ros::Publisher set_velocity_pub;
ros::Subscriber uav_state_sub;


// solver output
std::vector<double> x;   
std::vector<double> y;
std::vector<double> z;
std::vector<double> vx;
std::vector<double> vy;
std::vector<double> vz;

// trajectory to send to UAL
std::vector<double> x_ual;
std::vector<double> y_ual;
std::vector<double> z_ual;
std::vector<double> vx_ual;
std::vector<double> vy_ual;
std::vector<double> vz_ual;


bool solver_success; 
int start_point;
int ual_state;


/** callback for ual state
 */

void ualStateCallback(const uav_abstraction_layer::State::ConstPtr &msg){
    ual_state = msg->state;
}

/** Callback for the target pose
 */

void targetPoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    target_pose.pose = msg->pose.pose;
}

/** Thread to call UAL
 */
void UALthread(){
    
    ros::Rate rate(10);
    ROS_INFO("thread initialized");
    int cont = start_point;
    while(ros::ok()){
        if(!x_ual.empty() || !y_ual.empty() || !z_ual.empty()) // if there is no trajectory, the ual is not called
        {
            if(solver_success){ // if the solver success
                cont = start_point;
                solver_success = false;
            }
            else if(cont > time_horizon){ // if the trajectory is finished, nothing is done

            } 
            else cont = cont+1;

            if(!position_control || (cont>time_horizon)){
                /************** VELOCITY ***********************/
                clock_t time_now;
                time_now = clock() - initial_time;
                double time_taken = ((double)time_now)/CLOCKS_PER_SEC; // in seconds
                geometry_msgs::TwistStamped vel;
                vel.twist.linear.x = vx_ual[cont];
                vel.twist.linear.y = vy_ual[cont];
                vel.twist.linear.z = vz_ual[cont];
                vel.header.frame_id = "map";
                vel.header.stamp = ros::Time::now();
                set_velocity_pub.publish(vel);
                // TODO controlling yaw
            }
            /***************** POSITION CONTROL ****************************/
            else{
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = x_ual[cont];
                pose.pose.position.y = y_ual[cont];
                pose.pose.position.z = z_ual[cont];
                //publicar este punto

                geometry_msgs::PointStamped desired_point;
                desired_point.point.x = x_ual[cont];
                desired_point.point.y = y_ual[cont];
                desired_point.point.z = z_ual[cont];
                desired_point.header.frame_id = "map";

                desired_pose_publisher.publish(desired_point);
                std::cout<<"cont: "<<cont<<std::endl;

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
        
        }
        ros::spinOnce();
        rate.sleep();
    }
   
    
    

}

/** main thread
 */

int main(int _argc, char **_argv)
{
    ros::init(_argc, _argv, "optimal_control_interface_node");
    ros::NodeHandle nh = ros::NodeHandle();
    ros::NodeHandle pnh = ros::NodeHandle("~");
    int n_steps;
    float height_take_off;
    // parameters
    pnh.param<float>("height_take_off", height_take_off, 4.0);
    pnh.param<int>("start_point", start_point, 5);
    pnh.param<bool>("position_control", position_control, false);
    pnh.param<int>("n_steps_control", n_steps, 10);
    std::string target_topic;
    pnh.param<std::string>("target_topic",target_topic, "drc_vehicle_xp900/odometry");
    std::vector<double> desired_wp; 
    std::vector<double> desired_vel;
    std::vector<double> obst;

    if (ros::param::has("~desired_wp")) {
        ros::param::get("~desired_wp",desired_wp);
    }
    else {
        ROS_WARN("fail to get desidred wp");
    }

    if (ros::param::has("~desired_vel")) {
        ros::param::get("~desired_vel",desired_vel);
    }
    else {
        ROS_WARN("fail to get desired_vel");
    }
    if (ros::param::has("~no_fly_zone")) {
        ros::param::get("~no_fly_zone",obst);
    }
    else {
        ROS_WARN("fail to get no_fly_zone");
    }

    // subscribers and publishers
    ros::Subscriber target_pose_sub = nh.subscribe<nav_msgs::Odometry>(target_topic, 1, targetPoseCallback);
    ros::ServiceClient go_to_waypoint_srv_ = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>("uav_1/ual/go_to_waypoint");
    ros::ServiceClient take_off_srv = nh.serviceClient<uav_abstraction_layer::TakeOff>("drone_1/ual/take_off");
    set_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone_1/ual/set_pose",1);
    set_velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/drone_1/ual/set_velocity",1);
    desired_pose_publisher = pnh.advertise<geometry_msgs::PointStamped>("/desired_point",1);
    uav_state_sub = nh.subscribe<uav_abstraction_layer::State>("/drone_1/ual/state",1,ualStateCallback);
    // init solver
    init(nh);


    // taking off
    ROS_INFO("taking off");
    uav_abstraction_layer::TakeOff srv;
    srv.request.blocking = true;
    srv.request.height = height_take_off;
    if(!take_off_srv.call(srv)){
        ROS_WARN("the take off is not available");
    }
    while(ual_state!=uav_abstraction_layer::State::FLYING_AUTO)
    {   
        ROS_INFO("Taking off");
        ros::spinOnce();
        sleep(0.5);
    }
    ros::spinOnce();
    // thread for calling the UAL
    std::thread threadObjSolver(UALthread);

    ros::Rate rate(1); //hz
    /* main loop to call the solver. */
    while(ros::ok){   
        // solver function
          x.clear();
          y.clear();
          z.clear();
        solver_success = solverFunction(x,y,z,vx,vy,vz, desired_wp, desired_vel, obst);

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

        // log solver output to csv file
        logToCsv(x,y,z,vx,vy,vz,n_steps);

        // publish path to rviz visualizer
        publishPath(x,y,z,desired_wp);

        double point_1[2]= {-13.1,-35.55};
        double point_2[2]= {-2.2,-20.8};
        double point_3[2]= {10.77,-39.7};
        double point_4[2]= {-2.5,-51.3};

        publishNoFlyZone(point_1,point_2,point_3,point_4);

        publishDesiredPoint(desired_wp[0], desired_wp[1], desired_wp[2]);
        rate.sleep();
        ros::spinOnce();
    }
    
    threadObjSolver.join();
    return 0;
}
