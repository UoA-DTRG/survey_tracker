#ifndef __path_follower_mpc_H
#define __path_follower_mpc_H
#pragma once

#include "ros/ros.h"
#include "string.h"
#include <queue>
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "uav_messages/Heartbeat.h"
#include "uav_messages/TrajectoryFollowerStatus.h"
#include "mavros_msgs/State.h"
#include <tf/transform_listener.h>

#include <Eigen/Eigen>
#include <lapacke.h>
#include <../src/MPC/acado_common.h>
#include <../src/MPC/acado_auxiliary_functions.h>
#include "ringbuffer.hpp"
#include "flags.h"

using namespace std;

enum FOLLOWER_STATE{
    FOLLOWER_STATE_IDLE,
    FOLLOWER_STATE_FOLLOWING
};

struct path_follower_mpc_params{
    // Subscribe topics
    string trajectory_topic;
    string trajectory_splice_topic;

    // Publish topics
    string mavros_publish_topic;
    string current_path_index_topic;

    // Frames
    string odom_frame;
    string uav_frame;

    // Plant parameters
    Eigen::Vector3d tau;
    Eigen::Vector3d max_vel;
    Eigen::Vector3d max_delta_vel;

    // Weights for MPC control
    Eigen::Vector3d pos_weights;
    Eigen::Vector3d vel_weights;
    Eigen::Vector3d vel_setpoint_weights;
    Eigen::Vector3d vel_setpoint_delta_weights;

    // Yaw lookahead parameters
    int yaw_lookahead_start;
    int yaw_lookahead_end;

    // Yaw gain for P control
    double yaw_kp;
    // Max permissable yaw error before velocity is blocked
    double blocking_yaw_error;
    // Do not block velocity control if the commanded setpoint is smaller than this
    double blocking_min_vel;
    double blocking_min_vel_sqr;
    
    // Maximum RMS position error before a new trajectory can be started
    double max_new_traj_start_rms;
    double max_new_traj_start_rms_sqr;

    // Only check heartbeat if this is enabled
    bool heartbeat_enable;

    // Maximum poshold reset deviation
    double poshold_setpoint_reset_dist;

    // Transport delay between velocity input and execution
    double transport_delay;
};

class path_follower_mpc{
public:
    path_follower_mpc(tf::TransformListener* listener);

    void controller_tick(const ros::TimerEvent& event);
    void ping_planner_node(const ros::TimerEvent& event);

    // Blocking check for transform to avoid extrapolation issue.
    void check_for_odom_to_body_tf();

private:
    path_follower_mpc_params m_params;
    
    // Tf stuff
    tf::TransformListener* m_tf_listener;

    // Subscribers
    ros::Subscriber m_trajectory_subscriber;
    ros::Subscriber m_trajectory_splice_subscriber;
    ros::Subscriber m_angle_subscriber;
    ros::Subscriber m_velocity_subscriber;
    ros::Subscriber m_state_subscriber;

    // Publishers
    ros::Publisher m_velocity_setpoint_publisher;
    ros::Publisher m_current_path_index_publisher;
    ros::Publisher m_mandatory_finished_publisher;

    // Current index for syncing with path smoother
    std_msgs::Int32 m_current_path_index;

    // Trajectory tracking
    bool m_initial_position_initialized;
    void update_trajectory(const trajectory_msgs::MultiDOFJointTrajectory t);
    void splice_trajectory(const trajectory_msgs::MultiDOFJointTrajectory t);
    bool interp_traj(const double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d& acc);
    void set_traj_flags(trajectory_msgs::MultiDOFJointTrajectory* t);
    void handle_trajectory(trajectory_msgs::MultiDOFJointTrajectory t);
    trajectory_msgs::MultiDOFJointTrajectory m_trajectory_target;
    ros::Time m_trajectory_start_time;
    bool m_new_trajectory;
    double m_trajectory_temporal_resolution;
    Eigen::Vector3d m_trajectory_position_target;
    geometry_msgs::Twist m_target_velocity;
    FOLLOWER_STATE m_follower_state;
    bool m_trajectory_mandatory;
    bool m_trajectory_noyaw;
    queue<trajectory_msgs::MultiDOFJointTrajectory> m_trajectory_queue;

    // MPC stuff
    Eigen::Matrix<double, ACADO_NY, ACADO_NY> m_W_mat;
    Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> m_Wn_mat;     // Note: Wn_mat is the same as P, solution to CARE
    Eigen::MatrixXd solve_CARE();
    static lapack_logical select_lhp(const double *real, const double *imag) {return *real < 0.0;}
    Eigen::Vector3d calc_vel_setpoint(int traj_start_ind, Eigen::Vector3d x0_pos);
    Eigen::Vector3d calc_vel_setpoint(const double t, Eigen::Vector3d x0_pos);
    Eigen::Vector3d calc_vel_setpoint_stationary(Eigen::Vector3d x0_pos, Eigen::Vector3d target_pos);
    void initialize_acado_parameters();

    // A and B matricies
    Eigen::Matrix<double, 9, 9> m_A;
    Eigen::Matrix<double, 9, 3> m_B;

    // Current velocity estimates
    geometry_msgs::TwistStamped m_current_velocity_estimate;
    void update_velocity_estimate(const geometry_msgs::TwistStamped est);

    // Yaw tracking
    double m_target_yaw_angle;
    bool m_pure_yaw;
    vector<double> m_weighting_vector;
    void update_yaw_target(const std_msgs::Float64 yaw_target);
    double find_target_yaw(int current_index, Eigen::Vector3d here, bool &status);
    void populate_weighting_array();

    // Yaw stalling
    bool m_stall_trajectory_following;
    ros::Time m_stall_start_time;
    
    // Safety stuff
    bool m_trajectory_started;
    ros::ServiceClient m_heartbeat_client;

    // Status server
    ros::ServiceServer m_status_server;
    bool handle_status_service(uav_messages::TrajectoryFollowerStatus::Request &req, uav_messages::TrajectoryFollowerStatus::Response &res);

    // Flight mode tracking
    bool m_is_offboard;
    void handle_new_state_message(mavros_msgs::StateConstPtr state);

    // Trajectory start stabilize stalling
    RingBuffer<double>* m_x_ringbuf;
    RingBuffer<double>* m_y_ringbuf;
    RingBuffer<double>* m_z_ringbuf;
    Eigen::Vector3d last_pos {0, 0, 0};

    // Debug
    ros::Time m_last_run_time;

    // Initialization
    void init();
    void init_parameters();
    void init_subscribers();
    void init_publishers();
};

#endif