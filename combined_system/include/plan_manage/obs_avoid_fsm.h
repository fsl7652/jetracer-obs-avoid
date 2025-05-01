#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <mapping.h>
#include <plan_manage/traj_manager.h>

const double BUDGET = 0.5; // seconds

class ObsAvoidFSM {
    private:
    enum State {
        WAIT,
        START,
        EXECUTE,
        REPLAN,
        AT_GOAL
    } state = WAIT;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    MappingProcess::Ptr map;
    TrajPlanner::Ptr planner;
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber goal_sub, odom_sub;
    ros::Timer main_timer;


    bool have_goal, collision, traj_completed;
    Eigen::Vector4d pos;
    Eigen::Vector4d goal;
    Eigen::Vector2d cur_pos;
    double cur_yaw, cur_vel;
    double car_d_cr_;
    double budget_time;

    void OdomCallback(const nav_msgs::Odometry& msg);
    void FSMCallback(const ros::TimerEvent &e);
    void publishVel();
    void GoalCallback(const geometry_msgs::PoseStamped &msg);
    public:
    ObsAvoidFSM(): tf_listener_(tf_buffer_) {}
    ~ObsAvoidFSM()
    {
    }

    void init(ros::NodeHandle &nh);
    std::string odom_topic_ = "map";
    int returnState() const;
    void setGoal(geometry_msgs::PoseStamped goal);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

    
