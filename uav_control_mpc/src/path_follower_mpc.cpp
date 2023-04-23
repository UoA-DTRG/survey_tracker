#include "ros/ros.h"
#include "ros/spinner.h"
#include "ros/callback_queue.h"
#include "path_follower_mpc.h"

// This has to be done here to avoid the compiler throwing a fit :'(
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

// The following function was lifted from an ethz implementation of MPC
lapack_logical select_lhp(const double *real, const double *imag)
{
  return *real < 0.0;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "Controller_path_follower_mpc");

    tf::TransformListener listener;
    path_follower_mpc f = path_follower_mpc(&listener);

    // Blocking call to make sure the tf buffer is fed correctly before starting the timer
    //f.check_for_odom_to_body_tf();
    
    ros::Duration(1.0).sleep();

    ros::NodeHandle nh;
    ros::NodeHandle nh_heartbeat;
    ros::CallbackQueue heartbeat_queue;

    // Set nh_heartbeat to queue to a different queue
    nh_heartbeat.setCallbackQueue(&heartbeat_queue);

    ros::Timer timer = nh.createTimer(ros::Duration(0.02), &path_follower_mpc::controller_tick, &f);
    ros::Timer heartbeat_timer = nh_heartbeat.createTimer(ros::Duration(1.0), &path_follower_mpc::ping_planner_node, &f);

    //ros::spin();
    // Handle all normal control tasks in main_spinner and heartbeat in a separate spinner
    // This stops the blocking service call from killing control action when the planner node is lagging
    ros::AsyncSpinner main_spinner(1);
    ros::AsyncSpinner heartbeat_spinner(1, &heartbeat_queue);

    main_spinner.start();
    heartbeat_spinner.start();
    ros::waitForShutdown();
}