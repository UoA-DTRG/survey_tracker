#include <ros/ros.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "uav_messages/Heartbeat.h"
#include <Eigen/Eigen>
#include "helpers.hpp"

using namespace std;

constexpr double scaling_xy = 1.8;
constexpr double yaw = M_PI / 4;
constexpr double rotation = 0.1; // Approx 20 deg
constexpr double zoff = 1.5;
constexpr double path_length = 2 * M_PI * scaling_xy;
constexpr double speed = 3; // Target speed in m/s

bool handle_heartbeat(uav_messages::Heartbeat::Request &req, uav_messages::Heartbeat::Response &res){
    res.heartbeat = req.heartbeat;
    return true;
}

void getXYZ(double &x, double &y, double &z, double t){
    const double scale = 2 / (3 - cos(2 * t));
    x = scale * cos(t);
    y = scale * sin(2 * t) / 2;
    z = 0;
    //x = cos(t);
    //y = sin(t);
    //z = 0;
}

void populate_pos_vel(vector<Eigen::Vector3d> &points, vector<Eigen::Vector3d> &velocities, vector<Eigen::Vector3d> &accelerations, double target_mean_speed){
    const double time_factor = 2 * M_PI / (path_length / target_mean_speed);
    const double finish_time = (path_length / target_mean_speed);
    const double finish_time_sync = 3 * (((double)((int) (finish_time * 10) + 1)) / 10);

    for(size_t i = 0; i < 5; ++i){
        double x, y, z;
        getXYZ(x, y, z, 0);

        x *= scaling_xy;
        y *= scaling_xy;
        //z *= scaling_z;

        Eigen::Matrix3d rotm_y;
        rotm_y << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1;

        Eigen::Matrix3d rotm;
        rotm << cos(rotation), 0, -sin(rotation),
                0, 1, 0,
                sin(rotation), 0, cos(rotation);

        Eigen::Vector3d p;
        p << x, y, z;
        Eigen::Vector3d v = Eigen::Vector3d::Zero();

        p = rotm * p;
        p = rotm_y * p;

        p(2) += zoff;

        points.push_back(p);
        velocities.push_back(v);
        accelerations.push_back(Eigen::Vector3d::Zero());
    }

    Eigen::Vector3d vv(0, 0, 0);

    for(double now=0; now < finish_time_sync + 0.2; now+= 0.1){
        double t = now * time_factor;
        double tp1 = (now + 0.1) * time_factor;
        double x, y, z;
        getXYZ(x, y, z, t);

        x *= scaling_xy;
        y *= scaling_xy;
        //z *= scaling_z;

        double vx, vy, vz;
        getXYZ(vx, vy, vz, tp1);
        
        vx *= scaling_xy;
        vy *= scaling_xy;

        vx -= x;
        vy -= y;
        vz -= z;
        vx /= 0.1;
        vy /= 0.1;
        vz /= 0.1;

        //z+= zoff;

        Eigen::Matrix3d rotm_y;
        rotm_y << cos(yaw), -sin(yaw), 0,
                  sin(yaw), cos(yaw), 0,
                  0, 0, 1;

        Eigen::Matrix3d rotm;
        rotm << cos(rotation), 0, -sin(rotation),
                0, 1, 0,
                sin(rotation), 0, cos(rotation);

        Eigen::Vector3d p;
        p << x, y, z;

        p = rotm * p;
        p = rotm_y * p;
        p(2) += zoff;

        Eigen::Vector3d v;
        v << vx, vy, vz;
        v = rotm * v;
        v = rotm_y * v;

        Eigen::Vector3d a;
        a = (v - vv) / 0.1;
        vv = v;

        points.push_back(p);
        velocities.push_back(v);
        accelerations.push_back(a);
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "bspline_test_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>("/planner/vis", 5);
    ros::Publisher traj_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/uav/target/trajectory", 5);

    auto m_heartbeat_service = n.advertiseService("traj_follower_heartbeat", &handle_heartbeat);

    ros::spinOnce();
    
    vector<Eigen::Vector3d> points;
    vector<Eigen::Vector3d> velocities;
    vector<Eigen::Vector3d> accelerations;

    points.clear();
    velocities.clear();
    accelerations.clear();

    populate_pos_vel(points, velocities, accelerations, speed);

    printf("Traj of size %lu generated\n", points.size());

    for(size_t i = 0; i < points.size() - 1; ++i){
        //Eigen::Vector3d acc = velocities[i+1] - velocities[i];
        //accelerations.push_back(acc);
        //acc /= 0.1;
        auto acc = accelerations[i];
        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", points[i](0), points[i](1), points[i](2), (points[i+1] - points[i]).norm(), velocities[i](0), velocities[i](1), velocities[i](2), velocities[i].norm(), acc(0), acc(1), acc(2), acc.norm());
    }

    accelerations.push_back(Eigen::Vector3d(0, 0, 0));

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker m;
    m.header.stamp = ros::Time();
    m.header.frame_id = "map";
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.type = visualization_msgs::Marker::LINE_STRIP;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1.0;
    m.color.b = 0.0;
    m.color.g = 0.0;
    m.color.r = 1.0;
    m.color.a = 1.0;
    m.scale.x = 0.05;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.id = 0;

    m.points.clear();

    trajectory_msgs::MultiDOFJointTrajectory t;

    for(size_t i = 0; i < points.size() -1; ++i){
        auto p = points[i];
        auto v = velocities[i];
        auto a = accelerations[i];

        geometry_msgs::Transform pos;
        geometry_msgs::Point pos_p;
        geometry_msgs::Twist vel, acc;
        acc.linear.x = a(0);
        acc.linear.y = a(1);
        acc.linear.z = a(2);

        helpers::eigen_vector_to_msg(p, pos.translation);
        helpers::eigen_vector_to_msg(p, pos_p);
        helpers::eigen_vector_to_msg(v, vel.linear);

        trajectory_msgs::MultiDOFJointTrajectoryPoint tp;
        tp.time_from_start = ros::Duration((double) i * 0.1);
        
        tp.transforms.push_back(pos);
        tp.velocities.push_back(vel);
        tp.accelerations.push_back(acc);

        t.points.push_back(tp);

        m.points.push_back(pos_p);
    }

    ma.markers.push_back(m);

    bool pub = false;    

    while(ros::ok()){
        if(!pub){
            ros::Duration(1.0).sleep();
            traj_pub.publish(t);
            pub = true;
        }

        vis_pub.publish(ma);

        ros::spinOnce();
    }


    return 0;
}

