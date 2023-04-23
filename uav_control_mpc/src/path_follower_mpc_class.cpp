#include "path_follower_mpc.h"
#include "helpers.hpp"
#include "timer.hpp"

using namespace std;

path_follower_mpc::path_follower_mpc(tf::TransformListener* listener) : m_initial_position_initialized(false){
    m_tf_listener = listener;
    m_stall_trajectory_following = false;
    m_follower_state = FOLLOWER_STATE_IDLE;
    init();
}

void path_follower_mpc::init(){
    init_parameters();
    init_subscribers();
    init_publishers();
}

void path_follower_mpc::init_parameters(){
    ROS_INFO("Started MPC node");
    
    ros::NodeHandle pn("~");
    pn.param("trajectory_topic", m_params.trajectory_topic, string("/uav/target/trajectory"));
    pn.param("trajectory_splice_topic", m_params.trajectory_splice_topic, string("/uav/target/trajectory_splice"));
    pn.param("mavros_publish_topic", m_params.mavros_publish_topic, string("/mavros/setpoint_velocity/cmd_vel_unstamped"));
    pn.param("current_path_index_topic", m_params.current_path_index_topic, string("/uav/path/current_index"));

    pn.param("heartbeat_enable", m_params.heartbeat_enable, true);

    pn.param("odom_frame", m_params.odom_frame, string("odom_frame"));
    pn.param("uav_frame", m_params.uav_frame, string("fcu_link"));

    // pn.param("pos_kp_xy", m_params.pos_kp_xy, 2.0);
    // pn.param("pos_kp_z", m_params.pos_kp_z, 1.0);
    pn.param("yaw_kp", m_params.yaw_kp, 1.0);
    ROS_INFO("Using yaw Kp = %f\n", m_params.yaw_kp);
    // pn.param("max_pos_vel", m_params.max_pos_vel, 2.0);
    // m_params.max_pos_vel_sqr = m_params.max_pos_vel * m_params.max_pos_vel;

    // Get the poshold reset distance threshold from parameters
    pn.param("poshold_setpoint_reset_dist", m_params.poshold_setpoint_reset_dist, 99999.0);

    // Time constants of the plant. We will feed these into ACADO via OnlineData
    double tau;
    pn.param("tau_x", tau, 0.7488);
    m_params.tau(0) = tau;
    pn.param("tau_y", tau, 0.7223);
    m_params.tau(1) = tau;
    pn.param("tau_z", tau, 0.1575);
    m_params.tau(2) = tau;

    // Maximum velocity constraints. These are hard constrants that are experimentally verified
    double max_vel;
    pn.param("max_xy_vel", max_vel, 5.0);
    m_params.max_vel(0) = max_vel;
    m_params.max_vel(1) = max_vel;
    pn.param("max_z_vel", max_vel, 2.0);
    m_params.max_vel(2) = max_vel;
    
    // Max delta v constraints. This is for smoothness
    double max_delta_v;
    pn.param("max_delta_vel", max_delta_v, 4.0);
    m_params.max_delta_vel(0) = max_delta_v;
    m_params.max_delta_vel(1) = max_delta_v;
    m_params.max_delta_vel(2) = max_delta_v;

    // Weights for position control in MPC
    double W;
    pn.param("position_weight", W, 1.0);
    m_params.pos_weights(0) = W;
    m_params.pos_weights(1) = W;
    m_params.pos_weights(2) = W;

    // Weights for velocity control in MPC
    pn.param("velocity_weight", W, 0.1);
    m_params.vel_weights(0) = W;
    m_params.vel_weights(1) = W;
    m_params.vel_weights(2) = W;

    // Weights for velocity setpoint control in MPC
    pn.param("velocity_setpoint_weight", W, 0.1);
    m_params.vel_setpoint_weights(0) = W;
    m_params.vel_setpoint_weights(1) = W;
    m_params.vel_setpoint_weights(2) = W;

    pn.param("vel_setpoint_delta_weights", W, 0.5);
    m_params.vel_setpoint_delta_weights << W, W, W;

    Eigen::VectorXd zero(12);
    zero << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    m_W_mat = zero.asDiagonal();
    m_W_mat.block(0, 0, 3, 3) = m_params.pos_weights.asDiagonal();
    m_W_mat.block(3, 3, 3, 3) = m_params.vel_weights.asDiagonal();
    m_W_mat.block(6, 6, 3, 3) = m_params.vel_setpoint_weights.asDiagonal();
    m_W_mat.block(9, 9, 3, 3) = m_params.vel_setpoint_delta_weights.asDiagonal();

    m_A = Eigen::Matrix<double, 9, 9>::Zero();
    m_B = Eigen::Matrix<double, 9, 3>::Zero();

    // Set up termination weights for stability and convergence
    ROS_INFO("Solving CARE for termination weights ");
    m_Wn_mat = solve_CARE();

    ROS_INFO("Initializing ACADO solver");
    // Init ACADO solver
    acado_initializeSolver();

    // Set up remainder of solver parameters
    ROS_INFO("Initializing ACADO parameters");
    initialize_acado_parameters();

    pn.param("transport_delay", m_params.transport_delay, 0.1);

    // Yaw lookahead parameters
    pn.param("yaw_lookahead_start", m_params.yaw_lookahead_start, 1);
    pn.param("yaw_lookahead_end", m_params.yaw_lookahead_end, 20);

    // Populate the weighting array
    populate_weighting_array();

    // 0.61 rad is about 35 degrees
    pn.param("blocking_yaw_error", m_params.blocking_yaw_error, 0.61);
    pn.param("blocking_min_vel", m_params.blocking_min_vel, 0.2);
    m_params.blocking_min_vel_sqr = m_params.blocking_min_vel * m_params.blocking_min_vel;

    pn.param("max_new_traj_start_rms", m_params.max_new_traj_start_rms, 0.2);
    m_params.max_new_traj_start_rms_sqr = m_params.max_new_traj_start_rms * m_params.max_new_traj_start_rms;
    int ringbuf_size;
    pn.param("rms_ringbuf_size", ringbuf_size, 100);
    m_x_ringbuf = new RingBuffer<double>(ringbuf_size);
    m_y_ringbuf = new RingBuffer<double>(ringbuf_size);
    m_z_ringbuf = new RingBuffer<double>(ringbuf_size);

    m_pure_yaw = false;
    m_trajectory_started = false;
    m_is_offboard = true;

    m_trajectory_mandatory = false;
    m_trajectory_noyaw = false;

    ROS_INFO("Successfully initialized MPC node");
}

void path_follower_mpc::init_subscribers(){
    ros::NodeHandle n;
    m_trajectory_subscriber = n.subscribe(m_params.trajectory_topic, 5, &path_follower_mpc::update_trajectory, this);
    m_trajectory_splice_subscriber = n.subscribe(m_params.trajectory_splice_topic, 5, &path_follower_mpc::splice_trajectory, this);
    m_angle_subscriber = n.subscribe("/uav/target/yaw_angle", 5, &path_follower_mpc::update_yaw_target, this);
    m_velocity_subscriber = n.subscribe("/mavros/local_position/velocity_local", 5, &path_follower_mpc::update_velocity_estimate, this);
    m_state_subscriber = n.subscribe("/mavros/state", 5, &path_follower_mpc::handle_new_state_message, this);

    m_heartbeat_client = n.serviceClient<uav_messages::Heartbeat>("traj_follower_heartbeat");
    m_status_server = n.advertiseService("traj_follower_status", &path_follower_mpc::handle_status_service, this);
}

void path_follower_mpc::init_publishers(){
    ros::NodeHandle n;
    m_velocity_setpoint_publisher = n.advertise<geometry_msgs::Twist>(m_params.mavros_publish_topic, 5);
    m_current_path_index_publisher = n.advertise<std_msgs::Int32>(m_params.current_path_index_topic, 5);
    m_mandatory_finished_publisher = n.advertise<std_msgs::Int32>("planner/abort_finished", 1);
}

void path_follower_mpc::handle_new_state_message(mavros_msgs::StateConstPtr state){
    m_is_offboard = false;
    if(state->mode == "OFFBOARD"){
        m_is_offboard = true;
    }
}

void path_follower_mpc::ping_planner_node(const ros::TimerEvent& event){
    // Only ping for planner node if enabled
    if(!m_params.heartbeat_enable) return;

    if(m_trajectory_started){
        // Ping planner node via service
        
        uav_messages::Heartbeat heartbeat_req;
        heartbeat_req.request.heartbeat = true;

        if(!m_heartbeat_client.call(heartbeat_req)){
            ROS_ERROR("Failed to get heartbeat from planner node, aborting trajectory!");
            m_trajectory_start_time = ros::Time(0);
            m_follower_state = FOLLOWER_STATE_IDLE;
        }
    }
}

void path_follower_mpc::update_trajectory(const trajectory_msgs::MultiDOFJointTrajectory t){
    m_trajectory_started = true;

    // Check if the current trajectory is mandatory
    if(!m_trajectory_mandatory){
        handle_trajectory(t);
    }else{
        ROS_WARN("Executing mandatory trajectory. Adding new traj to queue");
        // The current trajectory is mandatory, we need to queue the request instead
        m_trajectory_queue.emplace(t);
    }
}

void path_follower_mpc::handle_trajectory(trajectory_msgs::MultiDOFJointTrajectory t){
    // Splice takes priority over new
    if(t.points[0].time_from_start > ros::Duration(0.01)){
        splice_trajectory(t);
        return;
    }

    // If trajectory is not mandatory then immediately start new one
    m_trajectory_target = t;
    m_new_trajectory = true;
    //m_trajectory_start_time = ros::Time::now();

    // Set set trajectory flags
    set_traj_flags(&t);

    // Set status to following as soon as a new trajectory is received
    m_follower_state = FOLLOWER_STATE_FOLLOWING;

    // Set yaw only flag to false to prevent false triggers
    m_stall_trajectory_following = false;

    // We can guess the temporal resolution by checking the time difference between the first and second samples of the trajectory
    m_trajectory_temporal_resolution = (t.points[1].time_from_start - t.points[0].time_from_start).toSec();
}

void path_follower_mpc::set_traj_flags(trajectory_msgs::MultiDOFJointTrajectory* t){
    // Assume flags are likely both false
    m_trajectory_mandatory = false;
    m_trajectory_noyaw = false;
    // No flags set, assume non mandatory and yaw required
    if(t->joint_names.size() < 1){
        ROS_ERROR("No trajectory flags found, skipping.+");
        return;
    }

    // Interpret joint_names(0) as flags
    try{
        ROS_WARN("Jointinfo 0: %s", t->joint_names[0].c_str());
        // Try to interpret first parameter as integer
        int bitmask = stoi(t->joint_names[0]);

        if(bitmask & PATH_FOLLOWER_FLAG_BITMASK_MANDATORY)  m_trajectory_mandatory = true;
        if(bitmask & PATH_FOLLOWER_FLAG_BITMASK_NOYAW)      m_trajectory_noyaw = true;

        ROS_WARN("Flag: %i, Mandatory: %i, Noyaw: %i", bitmask, m_trajectory_mandatory, m_trajectory_noyaw);

    }catch (const std::invalid_argument& ia) {
        std::cout << "Trajectory contains an invalid bitmask!: " << ia.what() << '\n';
    }
}

void path_follower_mpc::splice_trajectory(trajectory_msgs::MultiDOFJointTrajectory t){
    // Find the index to splice at
    size_t splice_index = 0;
    for(; splice_index < m_trajectory_target.points.size(); ++splice_index){
        const auto now = m_trajectory_target.points[splice_index].time_from_start;
        if(now > t.points[0].time_from_start){
            splice_index--;
            break;
        }
    }

    // Set Trajectory flags
    set_traj_flags(&t);

    // Resize trajectory target if necessary
    // if(m_trajectory_target.points.size() < (t.points.size() + splice_index)){
    //     m_trajectory_target.points.resize((t.points.size() + splice_index));
    // }
    m_trajectory_target.points.resize((t.points.size() + splice_index));

    for(int i = 0; i < t.points.size(); ++i){
        m_trajectory_target.points[splice_index + i] = t.points[i];
        //m_trajectory_target.points[splice_index].transforms[0] = t.points[splice_index].transforms[0];
        //m_trajectory_target.points[splice_index].velocities[0] = t.points[splice_index].velocities[0];
        //m_trajectory_target.points[splice_index].accelerations[0] = t.points[splice_index].accelerations[0];   
    }

    for(auto p : m_trajectory_target.points){
        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z,
                                                        p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z,
                                                        p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z);
    }
}

void path_follower_mpc::update_yaw_target(const std_msgs::Float64 yaw_target){
    ROS_INFO("Got new pure yaw target");
    m_pure_yaw = true;
    m_target_yaw_angle = yaw_target.data;
}

void path_follower_mpc::controller_tick(const ros::TimerEvent& event){
    // Get current time 
    auto now = ros::Time::now();
    auto diff = now - m_last_run_time;
    m_last_run_time = now;

    if(diff > ros::Duration(0.03)){
        ROS_ERROR("Timer deadline missed! Is the CPU load too high? %f", diff.toSec());
    }

    // If we haven't captured our location yet
    if(!m_initial_position_initialized){
        // Set the initial position so we don't drop back to the ground
        try{
            tf::StampedTransform odom_to_body;
            m_tf_listener->waitForTransform(m_params.odom_frame, m_params.uav_frame, now, ros::Duration(1.0));
            m_tf_listener->lookupTransform(m_params.odom_frame, m_params.uav_frame, now, odom_to_body);
            helpers::tf_to_eigen_vector(m_trajectory_position_target, odom_to_body);

            m_initial_position_initialized = true;
        }catch(tf::TransformException ex){
            ROS_INFO("Failed to lookup tf: %s", ex.what());
            return;
        }
    }

    // Check if this is a new trajectory, if it is then set the trajectory start time
    if(m_new_trajectory){
        // Check that the UAV is in a hover state before executing new trajectories unless the size is 1 (which indicates an abort trajectory)
        if((m_x_ringbuf->get_rms_sqr() < m_params.max_new_traj_start_rms_sqr &&
            m_y_ringbuf->get_rms_sqr() < m_params.max_new_traj_start_rms_sqr &&
            m_z_ringbuf->get_rms_sqr() < m_params.max_new_traj_start_rms_sqr) ||
            (m_trajectory_target.points.size() == 1)){
            // NOTE: A trajectory of size 1 will only ever be generated when the planner wants to abort
            // the currently executing trajectory immediately
            m_new_trajectory = false;
            m_trajectory_start_time = now;
        }else{
            ROS_INFO("RMS Criteria not met! Stalling start");
        }
    }

    // Lookup the transform between odom and body
    tf::StampedTransform odom_to_body;
    try{
        m_tf_listener->lookupTransform(m_params.odom_frame, m_params.uav_frame, ros::Time(0), odom_to_body);
    }catch (tf::TransformException ex){
        ROS_INFO("Failed to lookup tf: %s", ex.what());
        return;
    }

    // Get yaw
    tf::Quaternion q = odom_to_body.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Convert representation to eigen for easy calculations
    Eigen::Vector3d pos, velocity_setpoint;
    helpers::tf_to_eigen_vector(pos, odom_to_body);

    m_x_ringbuf->enqueue(pos(0) - m_trajectory_position_target(0));
    m_y_ringbuf->enqueue(pos(1) - m_trajectory_position_target(1));
    m_z_ringbuf->enqueue(pos(2) - m_trajectory_position_target(2));

    auto time_since_start = now - m_trajectory_start_time;
    int index = (time_since_start.toSec() / m_trajectory_temporal_resolution);

    // Reset our Twist object
    helpers::reset_twist_angular(m_target_velocity);
    int ind = m_trajectory_target.points.size() - 1;
    //auto traj_time = (ind >= 0) ? m_trajectory_target.points[ind].time_from_start : ros::Duration(0);
    auto traj_time = (ind >= 0) ? m_trajectory_target.points[m_trajectory_target.points.size() - 1].time_from_start : ros::Duration(0);
    double t1 = time_since_start.toSec(), t2 = traj_time.toSec();
    ROS_INFO("Time: %f Traj Time: %f Size: %lu", t1, t2, m_trajectory_target.points.size());
    if(time_since_start > ros::Duration(0) && time_since_start < traj_time && !m_new_trajectory){
        m_follower_state = FOLLOWER_STATE_FOLLOWING;
        // Disable yaw only if we receive a new trajectory and the yaw error is small
        m_pure_yaw = false;

        // We are still within the (achievable) time period from optimization
        // Pick the nearest point in the future and roll with it
        auto target_trajectory_point = m_trajectory_target.points[index];
        Eigen::Vector3d pos_error, pos_target;

        double time = time_since_start.toSec();
        
        velocity_setpoint = calc_vel_setpoint(time, pos);

        // This is NOT position error, we are simply doing this to avoid having to allocate an extra vector3d
        helpers::msg_to_eigen_vector(pos_target, target_trajectory_point.transforms[0].translation);

        // FIXME: Set position target to end of trajectory if we are close to it
        //if((traj_time.toSec() - time_since_start.toSec()) < 0.2) m_trajectory_position_target = pos_target;
        // Set pos target in case we need to immediately switch to poshold mode.
        m_trajectory_position_target = pos_target;

        // Calculate our desired yaw angle. We only do this in trajectory following mode and not position keep mode
        // Only do it if the velocity is large
        // Compute the yaw error based just on the target velocity. This will prevent "catch up" in position from
        // triggering our yaw blockage
        //double target_yaw = atan2(velocity_setpoint(1), velocity_setpoint(0));

        // Calculate our desired yaw angle by looking ahead to find the yaw angle that maximizes the visibility of
        // the next portion of the trajectory. This should let us avoid any surprises if we see an obstacle after
        // Moving through a corner
        bool status;
        double target_yaw = find_target_yaw(index, pos, status);

        double yaw_error = 0;
        // Only compute yaw setpoint if the trajectory requires it
        if(status && !m_trajectory_noyaw){
            yaw_error = helpers::find_min_rotation(target_yaw, yaw);
            ROS_INFO("Yaw err: %f", yaw_error);
            m_target_velocity.angular.z = yaw_error * m_params.yaw_kp;
        }else{
            // Disable yaw control if there isn't enough trajectory left to compute a setpoint correctly
            m_target_velocity.angular.z = 0;
        }
        
        // Correct our yaw before executing the trajectory if we are near the start and the trajectory is outside the fov
        //if(fabs(yaw_error) > m_params.blocking_yaw_error && velocity_setpoint.squaredNorm() > m_params.blocking_min_vel_sqr){
        if(fabs(yaw_error) > m_params.blocking_yaw_error){
            // If this is our first stall event, set the stall flag
            if(!m_stall_trajectory_following){
                m_stall_trajectory_following = true;
                m_stall_start_time = ros::Time::now();
            }

            // Simply run the position controller until we recover
            // Use MPC for position holding
            velocity_setpoint = calc_vel_setpoint_stationary(pos, m_trajectory_position_target);

        }else{
            // Check if we've just recovered from a yaw event
            if(m_stall_trajectory_following){
                // Disable the flag
                m_stall_trajectory_following = false;
                auto offset = ros::Time::now() - m_stall_start_time;
                // Update the start time to resync our trajectory
                m_trajectory_start_time += offset;
                // Recompute index so our current iteration is correct
                time_since_start = now - m_trajectory_start_time;
                index = (time_since_start.toSec() / m_trajectory_temporal_resolution);

                // We've corrected the index. Let the next cycle compute the correct pos/vel targets
                return;
            }
            // Update the position target
            m_trajectory_position_target = pos_target;

            // Publish the current waypoint index so the task earlier in the pipeline knows whats going on
            //ROS_INFO("Publishing index %i", index);
            m_current_path_index.data = index;
            m_current_path_index_publisher.publish(m_current_path_index);
        }
    }else{
        // Notify planner that mandatory trajectory is finished
        if(m_trajectory_mandatory && !m_new_trajectory){
            static std_msgs::Int32 dat;
            m_mandatory_finished_publisher.publish(dat);
        }
        // Check if the trajectory queue is empty and we haven't just ingested a new trajectory
        if(!m_trajectory_queue.empty() && !m_new_trajectory){
            // If the queue has stuff in it, dequeue and apply all sequentially
            // until either the queue is empty, or another mandatory trajectory is
            while(!m_trajectory_queue.empty()){
                // Deque front of queue
                auto t = m_trajectory_queue.front();
                m_trajectory_queue.pop();

                // Ingest the trajectory
                handle_trajectory(t);
                
                // m_trajectory_mandatory is set in handle_trajectory(t)
                // Stop ingesting data if the mandatory flag is set, or the queue is empty.
                if(m_trajectory_mandatory){
                    break;
                }
            }

            // MPC should be used to position hold for one cycle to avoid breaking the controller
            velocity_setpoint = calc_vel_setpoint_stationary(pos, m_trajectory_position_target);
        }else{
            m_follower_state = FOLLOWER_STATE_IDLE;
            // Check if the position deviation is larger than poshold_setpoint_reset_dist
            double deviation = (m_trajectory_position_target - pos).norm();
            if(deviation > m_params.poshold_setpoint_reset_dist || !m_is_offboard){
                //ROS_INFO("Resetting poshold setpoint");
                m_trajectory_position_target = pos;
            }

            // Use MPC for position holding
            velocity_setpoint = calc_vel_setpoint_stationary(pos, m_trajectory_position_target);
            
            // If we have a pure yaw setpoint, simply execute position hold and the setpoint
            if(m_pure_yaw){
                ROS_INFO("Executing pure yaw");
                double target_yaw = m_target_yaw_angle;
                double yaw_error = helpers::find_min_rotation(target_yaw, yaw);
                m_target_velocity.angular.z = yaw_error * m_params.yaw_kp;
            }
        }
    }

    // Publish our velocity setpoint
    //ROS_INFO("Publishing velocity setpoint: %f, %f, %f, %f", velocity_setpoint(0), velocity_setpoint(1), velocity_setpoint(2), m_target_velocity.angular.z);
    
    helpers::eigen_vector_to_msg(velocity_setpoint, m_target_velocity.linear);

    // Hack to disable yaw controller for now
    //m_target_velocity.angular.z = 0;

    m_velocity_setpoint_publisher.publish(m_target_velocity);
}

// The following code has been written according to this explanation
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.51.5104&rep=rep1&type=pdf
// Some of the code has been taken from an implementation by ethz-asl:
// https://github.com/ethz-asl/mav_control_rw/blob/master/mav_nonlinear_mpc/src/nonlinear_mpc.cc
Eigen::MatrixXd path_follower_mpc::solve_CARE(){
    m_A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    m_A.block(3, 3, 3, 3) = -1.0 * m_params.tau.asDiagonal();
    m_A.block(3, 6, 3, 3) = m_params.tau.asDiagonal();

    m_B.block(3, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);

    ROS_INFO("Constructed A and B matricies");

    Eigen::MatrixXd Q = m_W_mat.block(0, 0, 9, 9);
    Eigen::MatrixXd R = m_W_mat.block(9, 9, 3, 3);

    Eigen::MatrixXd Z;
    Z.resize(m_A.rows() + Q.rows(), m_A.cols() + m_A.cols());
    Z << m_A, -m_B * R.inverse() * m_B.transpose(), -Q, -m_A.transpose();

    ROS_INFO("Built Z matrix for CARE");

    int n = m_A.cols();
    Eigen::MatrixXd U(2 * n, 2 * n);  // Orthogonal matrix from Schur decomposition
    Eigen::VectorXd WR(2 * n);
    Eigen::VectorXd WI(2 * n);
    lapack_int sdim = 0;  // Number of eigenvalues for which sort is true
    lapack_int info;
    info = LAPACKE_dgees(LAPACK_COL_MAJOR,  // Eigen default storage order
        'V',               // Schur vectors are computed
        'S',               // Eigenvalues are sorted
        select_lhp,  // Ordering callback
        Z.rows(),          // Dimension of test matrix
        Z.data(),          // Pointer to first element
        Z.rows(),          // Leading dimension (column stride)
        &sdim,             // Number of eigenvalues sort is true
        WR.data(),         // Real portion of eigenvalues
        WI.data(),         // Complex portion of eigenvalues
        U.data(),          // Orthogonal transformation matrix
        Z.rows()           // Dimension of Z
    );         

    ROS_INFO("Solved for eigenvectors and eigenvalues");

    Eigen::MatrixXd U11 = U.block(0, 0, n, n).transpose();
    Eigen::MatrixXd U21 = U.block(n, 0, n, n).transpose();

    ROS_INFO("Computed U11 and U21");

    return U11.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(U21).transpose();
}

void path_follower_mpc::initialize_acado_parameters(){
    // I don't check here, but all weighting matricies should already be initialized here

    // Assign weights to solver
    //Eigen::Map<Eigen::Matrix<double, ACADO_NY, ACADO_NY>>(const_cast<double*>(acadoVariables.W)) = m_W_mat.transpose();
    //Eigen::Map<Eigen::Matrix<double, ACADO_NYN, ACADO_NYN>>(const_cast<double*>(acadoVariables.WN)) = m_Wn_mat.transpose();

    // Nested for go brrr
    for(size_t i=0; i<ACADO_NY; ++i){
        for(size_t j=0; j<ACADO_NY; ++j){
            acadoVariables.W[ACADO_NY * i + j] = m_W_mat(i, j);
        }
    }

    for(size_t i=0; i<ACADO_NYN; ++i){
        for(size_t j=0; j<ACADO_NYN; ++j){
            acadoVariables.WN[ACADO_NYN * i + j] = m_Wn_mat(i, j);
        }
    }

    ROS_INFO("Updated weighting matricies in ACADO solver");

    // Set up lower and upper bounds for the solver
    for(size_t i=0; i<ACADO_N; ++i){
        // Set lbound
        acadoVariables.lbValues[ACADO_NU * i + 0] = -m_params.max_delta_vel(0);         // Max x vel
        acadoVariables.lbValues[ACADO_NU * i + 1] = -m_params.max_delta_vel(1);     // Max y vel
        acadoVariables.lbValues[ACADO_NU * i + 2] = -m_params.max_delta_vel(2);     // Max z vel

        // Set ubound
        acadoVariables.ubValues[ACADO_NU * i + 0] = m_params.max_delta_vel(0);
        acadoVariables.ubValues[ACADO_NU * i + 1] = m_params.max_delta_vel(1);
        acadoVariables.ubValues[ACADO_NU * i + 2] = m_params.max_delta_vel(2);
    }

    for(size_t i = 0; i < ACADO_N; ++i){
        // Set lbound
        acadoVariables.lbAValues[ACADO_NU * i] = -m_params.max_vel(0);         // Max x vel
        acadoVariables.lbAValues[ACADO_NU * i + 1] = -m_params.max_vel(1);     // Max y vel
        acadoVariables.lbAValues[ACADO_NU * i + 2] = -m_params.max_vel(2);     // Max z vel

        // Set ubound
        acadoVariables.ubAValues[ACADO_NU * i] = m_params.max_vel(0);
        acadoVariables.ubAValues[ACADO_NU * i + 1] = m_params.max_vel(1);
        acadoVariables.ubAValues[ACADO_NU * i + 2] = m_params.max_vel(2);
    }

    ROS_INFO("Set Ubound and Lbound values");

    // Setup time constants as OnlineData
    for(size_t i=0; i<ACADO_N+1; ++i){
        acadoVariables.od[ACADO_NU * i] = m_params.tau(0);        // tau x
        acadoVariables.od[ACADO_NU * i + 1] = m_params.tau(1);    // tau y
        acadoVariables.od[ACADO_NU * i + 2] = m_params.tau(2);    // tau z
    }

    // Set x and u to 0
    for(size_t i=0; i < ACADO_N * ACADO_NX; ++i){
        acadoVariables.x[i] = 0;
    }

    for(size_t i=0; i < ACADO_N * ACADO_NU; ++i){
        acadoVariables.u[i] = 0;
    }
}

Eigen::Vector3d path_follower_mpc::calc_vel_setpoint(int traj_start_ind, Eigen::Vector3d x0_pos){
    // Time solver
    tick();

    // Predict our system state transport_delay seconds in the future
    Eigen::Vector3d p0_dot;
    Eigen::Vector3d u0;
    Eigen::Vector3d u0_dot = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 9, 1> x0;

    // Get current system state
    helpers::msg_to_eigen_vector(p0_dot, m_current_velocity_estimate.twist.linear);
    helpers::msg_to_eigen_vector(u0, m_target_velocity.linear);
    //u0_dot << acadoVariables.u[0], acadoVariables.u[0], acadoVariables.u[0];
    x0 << x0_pos, p0_dot, u0;

    // Predict system state using simple SS model
    auto x0_dot = m_A * m_params.transport_delay * x0 + m_B * m_params.transport_delay * u0_dot;
    x0 = x0 + x0_dot;
        
    // Set current state estimate
    // Position state estimates
    /*acadoVariables.x0[0] = x0_pos(0);
    acadoVariables.x0[1] = x0_pos(1);
    acadoVariables.x0[2] = x0_pos(2);
    
    // Velocity state estimates
    acadoVariables.x0[3] = m_current_velocity_estimate.twist.linear.x;
    acadoVariables.x0[4] = m_current_velocity_estimate.twist.linear.y;
    acadoVariables.x0[5] = m_current_velocity_estimate.twist.linear.z;

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = m_target_velocity.linear.x;
    acadoVariables.x0[7] = m_target_velocity.linear.y;
    acadoVariables.x0[8] = m_target_velocity.linear.z;*/

    acadoVariables.x0[0] = x0(0);
    acadoVariables.x0[1] = x0(1);
    acadoVariables.x0[2] = x0(2);

    // Velocity state estimates
    acadoVariables.x0[3] = x0(3);
    acadoVariables.x0[4] = x0(4);
    acadoVariables.x0[5] = x0(5);

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = x0(6);
    acadoVariables.x0[7] = x0(7);
    acadoVariables.x0[8] = x0(8);
    
    for(size_t i=0; i < ACADO_N; ++i){
        bool zero_target_vel = false;
        size_t ind = traj_start_ind + i;
        if(ind >= m_trajectory_target.points.size()){
            ind = m_trajectory_target.points.size() - 1;
            zero_target_vel = true;
        }

        // Position targets
        acadoVariables.y[i * (ACADO_NX + ACADO_NU)] = m_trajectory_target.points[ind].transforms[0].translation.x;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 1] = m_trajectory_target.points[ind].transforms[0].translation.y;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 2] = m_trajectory_target.points[ind].transforms[0].translation.z;

        // Velocity targets. We feed these forwards from our planner
        if(zero_target_vel){
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 3] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 4] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 5] = 0;

            // Velocity input targets. We set these to 0 to minimize our actual velocity
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 6] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 7] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 8] = 0;

            // Delta velocity targets. Always put 0 here to promote little change in velocity
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 9] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 10] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 11] = 0;
        }else{
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 3] = m_trajectory_target.points[ind].velocities[0].linear.x;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 4] = m_trajectory_target.points[ind].velocities[0].linear.y;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 5] = m_trajectory_target.points[ind].velocities[0].linear.z;

            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 6] = m_trajectory_target.points[ind].velocities[0].linear.x;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 7] = m_trajectory_target.points[ind].velocities[0].linear.y;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 8] = m_trajectory_target.points[ind].velocities[0].linear.z;

            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 9] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 10] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 11] = 0;
        }
    }

    // Set up terminal constraints
    bool zero_target_vel = false;
    size_t ind = traj_start_ind + ACADO_N + 1;
    if(ind >= m_trajectory_target.points.size()){
        ind = m_trajectory_target.points.size() - 1;
        zero_target_vel = true;
    }

    // Position targets
    acadoVariables.yN[0] = m_trajectory_target.points[ind].transforms[0].translation.x;
    acadoVariables.yN[1] = m_trajectory_target.points[ind].transforms[0].translation.y;
    acadoVariables.yN[2] = m_trajectory_target.points[ind].transforms[0].translation.z;

    // Velocity targets. We feed these forwards from our planner
    if(zero_target_vel){
        acadoVariables.yN[3] = 0;
        acadoVariables.yN[4] = 0;
        acadoVariables.yN[5] = 0;

        // Optimize output sets 3:5 and 6:8 to be the same, as they are velocity and velocity input
        acadoVariables.yN[6] = 0;
        acadoVariables.yN[7] = 0;
        acadoVariables.yN[8] = 0;
    }else{
        acadoVariables.yN[3] = m_trajectory_target.points[ind].velocities[0].linear.x;
        acadoVariables.yN[4] = m_trajectory_target.points[ind].velocities[0].linear.y;
        acadoVariables.yN[5] = m_trajectory_target.points[ind].velocities[0].linear.z;

        acadoVariables.yN[6] = 0;
        acadoVariables.yN[7] = 0;
        acadoVariables.yN[8] = 0;
    }

    // Run solver
    acado_preparationStep();
    acado_feedbackStep();

    ROS_INFO("MPC solve completed in %lu microseconds", tock());

    // TODO : Validate MPC output results as valid before executing them

    Eigen::Vector3d command_vel;
    command_vel << acadoVariables.x[15], acadoVariables.x[16], acadoVariables.x[17];

    return command_vel;
}

bool path_follower_mpc::interp_traj(const double t, Eigen::Vector3d &pos, Eigen::Vector3d &vel, Eigen::Vector3d &acc){
    for(size_t i = 0; i < m_trajectory_target.points.size() - 1; ++i){
        if(t < m_trajectory_target.points[i + 1].time_from_start.toSec()){
            Eigen::Vector3d temp_lo, temp_hi;
            double traj_time = m_trajectory_target.points[i + 1].time_from_start.toSec() 
                                - m_trajectory_target.points[i].time_from_start.toSec();
            double dt = t - m_trajectory_target.points[i].time_from_start.toSec();
            helpers::msg_to_eigen_vector(temp_lo, m_trajectory_target.points[i].transforms[0].translation);
            helpers::msg_to_eigen_vector(temp_hi, m_trajectory_target.points[i + 1].transforms[0].translation);

            // Linear interp
            pos = ((dt / traj_time) * (temp_hi - temp_lo)) + temp_lo;

            helpers::msg_to_eigen_vector(temp_lo, m_trajectory_target.points[i].velocities[0].linear);
            helpers::msg_to_eigen_vector(temp_hi, m_trajectory_target.points[i + 1].velocities[0].linear);
            vel = ((dt / traj_time) * (temp_hi - temp_lo)) + temp_lo;

            helpers::msg_to_eigen_vector(temp_lo, m_trajectory_target.points[i].accelerations[0].linear);
            helpers::msg_to_eigen_vector(temp_hi, m_trajectory_target.points[i + 1].accelerations[0].linear);
            acc = ((dt / traj_time) * (temp_hi - temp_lo)) + temp_lo;

            return true;
        }
    }

    auto point_last = &m_trajectory_target.points[m_trajectory_target.points.size() - 1];
    helpers::msg_to_eigen_vector(pos, point_last->transforms[0].translation);
    helpers::msg_to_eigen_vector(vel, point_last->velocities[0].linear);

    return false;
}

Eigen::Vector3d path_follower_mpc::calc_vel_setpoint(const double t, Eigen::Vector3d x0_pos){
     // Time solver
    tick();

    // Predict our system state transport_delay seconds in the future
    Eigen::Vector3d p0_dot;
    Eigen::Vector3d u0;
    Eigen::Vector3d u0_dot = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 9, 1> x0;

    // Get current system state
    helpers::msg_to_eigen_vector(p0_dot, m_current_velocity_estimate.twist.linear);
    helpers::msg_to_eigen_vector(u0, m_target_velocity.linear);
    //u0_dot << acadoVariables.u[0], acadoVariables.u[0], acadoVariables.u[0];
    x0 << x0_pos, p0_dot, u0;

    // Predict system state using simple SS model
    auto x0_dot = m_A * m_params.transport_delay * x0 + m_B * m_params.transport_delay * u0_dot;
    x0 = x0 + x0_dot;
        
    // Set current state estimate
    // Position state estimates
    /*acadoVariables.x0[0] = x0_pos(0);
    acadoVariables.x0[1] = x0_pos(1);
    acadoVariables.x0[2] = x0_pos(2);
    
    // Velocity state estimates
    acadoVariables.x0[3] = m_current_velocity_estimate.twist.linear.x;
    acadoVariables.x0[4] = m_current_velocity_estimate.twist.linear.y;
    acadoVariables.x0[5] = m_current_velocity_estimate.twist.linear.z;

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = m_target_velocity.linear.x;
    acadoVariables.x0[7] = m_target_velocity.linear.y;
    acadoVariables.x0[8] = m_target_velocity.linear.z;*/

    acadoVariables.x0[0] = x0(0);
    acadoVariables.x0[1] = x0(1);
    acadoVariables.x0[2] = x0(2);

    // Velocity state estimates
    acadoVariables.x0[3] = x0(3);
    acadoVariables.x0[4] = x0(4);
    acadoVariables.x0[5] = x0(5);

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = x0(6);
    acadoVariables.x0[7] = x0(7);
    acadoVariables.x0[8] = x0(8);

    // Set current state estimate
    // Position state estimates
    /*acadoVariables.x0[0] = x0_pos(0);
    acadoVariables.x0[1] = x0_pos(1);
    acadoVariables.x0[2] = x0_pos(2);
    
    // Velocity state estimates
    acadoVariables.x0[3] = m_current_velocity_estimate.twist.linear.x;
    acadoVariables.x0[4] = m_current_velocity_estimate.twist.linear.y;
    acadoVariables.x0[5] = m_current_velocity_estimate.twist.linear.z;

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = m_target_velocity.linear.x;
    acadoVariables.x0[7] = m_target_velocity.linear.y;
    acadoVariables.x0[8] = m_target_velocity.linear.z;*/
    
    for(size_t i=0; i < ACADO_N; ++i){
        Eigen::Vector3d pos_interp, vel_interp, acc_interp;
        bool zero_target_vel = interp_traj(t + (((double) i) * m_trajectory_temporal_resolution), pos_interp, vel_interp, acc_interp);

        // Position targets
        acadoVariables.y[i * (ACADO_NX + ACADO_NU)] = pos_interp(0);
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 1] = pos_interp(1);
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 2] = pos_interp(2);

        // Velocity targets. We feed these forwards from our planner
        if(zero_target_vel){
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 3] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 4] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 5] = 0;

            // Velocity input targets. We set these to 0 to minimize our actual velocity
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 6] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 7] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 8] = 0;

            // Delta velocity targets. Always put 0 here to promote little change in velocity
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 9] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 10] = 0;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 11] = 0;
        }else{
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 3] = vel_interp(0);
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 4] = vel_interp(1);
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 5] = vel_interp(2);

            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 6] = vel_interp(0);
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 7] = vel_interp(1);
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 8] = vel_interp(2);

            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 9] = acc_interp(0) * m_trajectory_temporal_resolution;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 10] = acc_interp(0) * m_trajectory_temporal_resolution;
            acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 11] = acc_interp(0) * m_trajectory_temporal_resolution;
        }
    }

    // Set up terminal constraints
    Eigen::Vector3d pos_interp, vel_interp, acc_interp;
    bool zero_target_vel = interp_traj(t + (((double) ACADO_N) * m_trajectory_temporal_resolution), pos_interp, vel_interp, acc_interp);

    // Position targets
    acadoVariables.yN[0] = pos_interp(0);
    acadoVariables.yN[1] = pos_interp(1);
    acadoVariables.yN[2] = pos_interp(2);

    // Velocity targets. We feed these forwards from our planner
    if(zero_target_vel){
        acadoVariables.yN[3] = 0;
        acadoVariables.yN[4] = 0;
        acadoVariables.yN[5] = 0;

        // Optimize output sets 3:5 and 6:8 to be the same, as they are velocity and velocity input
        acadoVariables.yN[6] = 0;
        acadoVariables.yN[7] = 0;
        acadoVariables.yN[8] = 0;
    }else{
        acadoVariables.yN[3] = vel_interp(0);
        acadoVariables.yN[4] = vel_interp(1);
        acadoVariables.yN[5] = vel_interp(2);

        acadoVariables.yN[6] = vel_interp(0);
        acadoVariables.yN[7] = vel_interp(1);
        acadoVariables.yN[8] = vel_interp(2);
    }

    // Run solver
    acado_preparationStep();
    acado_feedbackStep();

    ROS_INFO("MPC solve completed in %lu microseconds", tock());

    // TODO : Validate MPC output results as valid before executing them

    Eigen::Vector3d command_vel;
    command_vel << acadoVariables.x[15], acadoVariables.x[16], acadoVariables.x[17];

    return command_vel;
}

Eigen::Vector3d path_follower_mpc::calc_vel_setpoint_stationary(Eigen::Vector3d x0_pos, Eigen::Vector3d target_pos){
    // Time solver
    tick();

    // Set current state estimate
    // Position state estimates
    acadoVariables.x0[0] = x0_pos(0);
    acadoVariables.x0[1] = x0_pos(1);
    acadoVariables.x0[2] = x0_pos(2);
    
    // Velocity state estimates
    acadoVariables.x0[3] = m_current_velocity_estimate.twist.linear.x;
    acadoVariables.x0[4] = m_current_velocity_estimate.twist.linear.y;
    acadoVariables.x0[5] = m_current_velocity_estimate.twist.linear.z;

    // Set the initial velocity command to our last velocity command
    acadoVariables.x0[6] = m_target_velocity.linear.x;
    acadoVariables.x0[7] = m_target_velocity.linear.y;
    acadoVariables.x0[8] = m_target_velocity.linear.z;
    
    for(size_t i=0; i < ACADO_N; ++i){
        // Position targets
        acadoVariables.y[i * (ACADO_NX + ACADO_NU)] = target_pos(0);
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 1] = target_pos(1);
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 2] = target_pos(2);

        // Set velocity targets to 0 to encourage low control input stationary position tracking
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 3] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 4] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 5] = 0;

        // Velocity input targets. We set these to 0 to minimize our actual velocity
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 6] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 7] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 8] = 0;

        // Delta velocity targets. Always put 0 here to promote little change in velocity
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 9] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 10] = 0;
        acadoVariables.y[i * (ACADO_NX + ACADO_NU) + 11] = 0;
    }

    // Set up terminal constraints
    // Position targets
    acadoVariables.yN[0] = target_pos(0);
    acadoVariables.yN[1] = target_pos(1);
    acadoVariables.yN[2] = target_pos(2);

    // Velocity targets. We feed these forwards from our planner
    acadoVariables.yN[3] = 0;
    acadoVariables.yN[4] = 0;
    acadoVariables.yN[5] = 0;

    // Try and optimize deltas to be 0
    acadoVariables.yN[6] = 0;
    acadoVariables.yN[7] = 0;
    acadoVariables.yN[8] = 0;

    // Run solver
    acado_preparationStep();
    acado_feedbackStep();

    //ROS_INFO("MPC position solve completed in %lu microseconds", tock());

    // TODO : Validate MPC output results as valid before executing them

    Eigen::Vector3d command_vel;
    //command_vel << acadoVariables.u[0], acadoVariables.u[1], acadoVariables.u[2];
    command_vel << acadoVariables.x[15], acadoVariables.x[16], acadoVariables.x[17];


    return command_vel;
}

double path_follower_mpc::find_target_yaw(int current_index, Eigen::Vector3d here, bool &status){
    tick();
    int size = m_params.yaw_lookahead_end - m_params.yaw_lookahead_start;
    double yaw_angles[size];
    int count = 0;
    
    // Get yaw angle to the lookahead points
    for(int i=0; i<size; ++i){
        int ind = i + current_index + m_params.yaw_lookahead_start;
        if(ind >= m_trajectory_target.points.size()) break;
        count++;
        double dx, dy;
        dx = m_trajectory_target.points[ind].transforms[0].translation.x - here(0);
        dy = m_trajectory_target.points[ind].transforms[0].translation.y - here(1);
        yaw_angles[i] = atan2(dy, dx);
    }

    if(count < (size / 3)){
        status = false;
        return 0;
    }

    // Double check that we don't have opposite polarities in the +-pi region.
    // This will seriously mess with our calcs and break the controller for one timestep
    // We should never really see a case where the yaw angle changes by more than 12 rads/s
    // I'm happy to take the trajectory stall penalty in the event we do though.
    constexpr double flip_threshold = 5;
    double min = 10, max = -10;
    for(size_t i=0; i < size; ++i){
        if(yaw_angles[i] < min) min = yaw_angles[i];
        if(yaw_angles[i] > max) max = yaw_angles[i];
    }

    if((max - min)  > flip_threshold){
        for(size_t i=0; i<size; ++i){
            if(yaw_angles[i] < 0) yaw_angles[i] += 2 * M_PI;
        }
    }

    double div = 0;
    double tot = 0;

    for(size_t i=0; i<count; ++i){
        div += m_weighting_vector[i];
        tot += yaw_angles[i] * m_weighting_vector[i];
    }

    double ret = tot / div;

    // Ret will never be < pi. We only have to do this because we adjusted for sign earlier
    if(ret > M_PI) ret -= 2 * M_PI;

    printf("Target yaw: %f, took %lu microseconds\n", ret, tock());
    
    // Yaw error will return nan if we are at the end of the trajectory. We disable the controller instead here
    if(isnan(ret)) ret = 0;

    status = true;
    return ret;
}

void path_follower_mpc::populate_weighting_array(){
    int size = m_params.yaw_lookahead_end - m_params.yaw_lookahead_start;
    for(size_t i=0; i < size; ++i){
        m_weighting_vector.push_back(1.0);
    }
}

void path_follower_mpc::update_velocity_estimate(const geometry_msgs::TwistStamped est){
    m_current_velocity_estimate = est;
}

bool path_follower_mpc::handle_status_service(uav_messages::TrajectoryFollowerStatus::Request &req, uav_messages::TrajectoryFollowerStatus::Response &res){
    if(m_follower_state == FOLLOWER_STATE_IDLE){
        res.status = uav_messages::TrajectoryFollowerStatus::Request::IDLE;
    }else{
        res.status = uav_messages::TrajectoryFollowerStatus::Request::FOLLOWING;
    }

    return true;
}