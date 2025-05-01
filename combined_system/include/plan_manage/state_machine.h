#ifndef STATE_MACHINE
#define STATE_MACHINE

#pragma once
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <plan_manage/obs_avoid_fsm.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <Eigen/Eigen>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Controller {
    public:
        Controller(ros::NodeHandle& nh);
        void controlLoop();
    
    private:
        ros::NodeHandle nh_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        ObsAvoidFSM obs_avoid_fsm;

        ros::Publisher ml_road_follow_pub;
        ros::Publisher obs_goal_pub;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher scan_vis_pub;
        ros::Subscriber lidar_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber ml_conf_sub;
        ros::Subscriber steer_sub;

        Eigen::Vector2d cur_pos_;
        Eigen::Vector2d right_obs_pos_;
        Eigen::Vector2d left_obs_pos_;
        Eigen::Vector2d obs_pos_;

        double cur_ang_vel_;
        double cur_vel_;
        double cur_yaw_;
        double obs_angle_;
        double avoid_dir;
        double steer_gain;
        double rf_speed;

        bool obs_detect;
        bool pub_goal;
        bool is_confident;

        double furthest_left;
        double furthest_right;
        double closest_range;

        const double ANG_VEL_THRESHOLD = 0.15;
        const int MAX_SCAN_ANGLE = 50;

        int min_angle, max_angle;

        bool pot_obs_;

        void confCallback(const std_msgs::Bool::ConstPtr& msg);
        
        void toggleMLRoadFollow();
        void publishCmdVel(double linear_vel, double angular_vel);
        void publishObsAvoidGoal();
        void obsAvoid();
        void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void steerCallback(const std_msgs::Float32::ConstPtr& msg);
        void laneRecovery();
        void scanVis(double min_angle, double max_angle, double range);
        void emergencyStop();
    

        enum State { LANE_FOLLOW, OBS_AVOID, LANE_RECOVERY, STOP };
        State state;
};

#endif
