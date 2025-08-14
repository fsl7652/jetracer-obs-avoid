#include <plan_manage/state_machine.h>

Controller::Controller(ros::NodeHandle& nh) : nh_(nh), state(LANE_FOLLOW), tf_listener_(tf_buffer_){
    nh_.param("/road_follow/steer_gain", steer_gain, -0.9);
    nh_.param("/road_follow/linear_vel", rf_speed, 0.15);
    obs_avoid_fsm.init(nh_);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    lidar_sub = nh_.subscribe("/scan", 10, &Controller::lidarCallback, this);
    odom_sub = nh_.subscribe("/odom", 10, &Controller::odomCallback, this);
    ml_road_follow_pub = nh_.advertise<std_msgs::Bool>("/ml_enable", 1);
    obs_goal_pub = nh_.advertise<geometry_msgs::PoseStamped>("/goal_vis",1);
    ml_conf_sub =  nh_.subscribe("/conf_check",1,&Controller::confCallback,this);
    scan_vis_pub = nh_.advertise<visualization_msgs::Marker>("/scan_vis",1);
    steer_sub = nh_.subscribe("/steering_value", 10, &Controller::steerCallback, this);
}


void Controller::steerCallback(const std_msgs::Float32::ConstPtr& msg){
    if (state == LANE_FOLLOW){
        double steering = msg->data * steer_gain;
        publishCmdVel(rf_speed, steering);
    }
}



void Controller::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
        cur_pos_(0) = transformStamped.transform.translation.x;
        cur_pos_(1) = transformStamped.transform.translation.y;
        tf::Quaternion q(
    transformStamped.transform.rotation.x,
    transformStamped.transform.rotation.y,
    transformStamped.transform.rotation.z,
    transformStamped.transform.rotation.w);
        cur_yaw_ = tf::getYaw(q);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Lookup failed: %s", ex.what());
        return;
    }

    cur_vel_ = msg->twist.twist.linear.x;
    cur_ang_vel_ = msg->twist.twist.angular.z;
}

void Controller::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    obs_detect = false;
    pot_obs_ = false;
    furthest_left = 100.0;  
    furthest_right = -100.0; 
    closest_range = 12.0;
    obs_angle_ = 0.0;
    obs_pos_.setZero();
    left_obs_pos_.setZero();
    right_obs_pos_.setZero();
    if (state == LANE_FOLLOW || state == LANE_RECOVERY) {
        if (std::abs(cur_ang_vel_) <= ANG_VEL_THRESHOLD) {
            min_angle = -30;
            max_angle = 30;
        } 
        else if (cur_ang_vel_ > ANG_VEL_THRESHOLD) {
            double turn = std::min(cur_ang_vel_ / 0.2, 1.0);
            max_angle = static_cast<int>(std::round(25.0 * (1.0 + turn)));
            max_angle = std::min(max_angle, MAX_SCAN_ANGLE);
            min_angle = 0;
        } 
        else {
            double turn = std::min(std::abs(cur_ang_vel_) / 0.2, 1.0);
            min_angle = static_cast<int>(std::round(-25.0 * (1.0 + turn)));
            min_angle = std::max(min_angle, -MAX_SCAN_ANGLE);
            max_angle = 0;
        }
        //limit scan data to min_angle < x < max_angle
        double min_angle_rad = min_angle * M_PI / 180.0;
        double max_angle_rad =  max_angle * M_PI / 180.0;
        scanVis(min_angle_rad, max_angle_rad, 0.9);
        int num_ranges = msg->ranges.size();
        int min_idx = (static_cast<int>(round(min_angle_rad / msg->angle_increment)) + num_ranges) % num_ranges;
        int max_idx = (static_cast<int>(round(max_angle_rad / msg->angle_increment)) + num_ranges) % num_ranges;
        //ROS_INFO("MIN_IDX: %d  MAX_IDX: %d", min_idx,max_idx);
        for (int i = min_idx; i != max_idx; i = (i + 1) % num_ranges) {
            if (msg->ranges[i] <= 0.75 && msg->ranges[i] >= 0.16){
                pot_obs_ = true;
                break;
            }
            else {
                pot_obs_ = false;
            }
        }
        if(pot_obs_){
            //expanded ranges search to ensure all of obs is detected
           for (int i = min_idx; i != max_idx; i = (i + 1) % num_ranges) {
            if (msg->ranges[i] <= 0.90 && msg->ranges[i] >= 0.16) {
                obs_detect = true;
                double angle = i * msg->angle_increment;
                if (angle > M_PI) {
                    angle = angle - (2*M_PI);
                }
                if (angle < furthest_left) {
                    furthest_left = angle;
                    left_obs_pos_(0) = cos(angle) * msg->ranges[i];
                    left_obs_pos_(1) = sin(angle) * msg->ranges[i];
                }
                if (angle > furthest_right) {
                    furthest_right = angle;
                    right_obs_pos_(0) = cos(angle) * msg->ranges[i];
                    right_obs_pos_(1) = sin(angle) * msg->ranges[i];
                }
                if (msg->ranges[i] < closest_range) {
                    closest_range = msg->ranges[i];
                    obs_angle_ = angle;
                    obs_pos_(0) = cos(angle) * msg->ranges[i];
                    obs_pos_(1) = sin(angle) * msg->ranges[i];
                }
            }
            } 
        }
        if (closest_range < 0.25 && pot_obs_ && obs_detect){
          state = STOP;
        }
        else if (obs_detect){
            pub_goal = false;
            if (furthest_left < -0.2) { 
                avoid_dir = -1.0;
                ROS_INFO("Obstacle right, avoiding to left side");
            }
            else if (furthest_right > 0.2) { 
                avoid_dir = 1.0;
                ROS_INFO("Obstacle left, avoiding to right side");
            }
            else {
                avoid_dir = (furthest_left < -furthest_right) ? -1.0 : 1.0;
                ROS_INFO("Obstacle central, choosing a side");
            }
            state = OBS_AVOID;
        }

    }
}

void Controller::scanVis(double min_angle, double max_angle, double range){
    visualization_msgs::Marker cone;
    cone.header.frame_id = "base_link";
    cone.header.stamp = ros::Time::now();
    cone.ns = "scan_vis";
    cone.id = 0;
    cone.type = visualization_msgs::Marker::LINE_STRIP;
    cone.action = visualization_msgs::Marker::ADD;
    cone.pose.orientation.w = 1.0;
    cone.scale.x = 0.05;
    cone.color.a = 1.0;
    cone.color.r = 0.0;
    cone.color.g = 1.0;
    cone.color.b = 0.0;
    
    geometry_msgs::Point start;
    start.x = 0.0;
    start.y = 0.0;
    start.z = 0.0;
    
    cone.points.push_back(start);
    int line_num = 10;
    for (int i = 0; i <= line_num; i++){
        double angle = min_angle + i * (max_angle - min_angle) / line_num;
        geometry_msgs::Point p;
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        p.z = 0.0;
        cone.points.push_back(p);
    }
    cone.points.push_back(start);
    //ROS_INFO("Publishing scan window marker with angles %.2f to %.2f", min_angle, max_angle);
    scan_vis_pub.publish(cone);
    
}

void Controller::confCallback(const std_msgs::Bool::ConstPtr& msg){
    is_confident = msg->data;
    if (!is_confident) {
        toggleMLRoadFollow();
        state = STOP;
    }
}

void Controller::toggleMLRoadFollow() {
    std_msgs::Bool enable_msg;
    enable_msg.data = (state == LANE_FOLLOW);
    ml_road_follow_pub.publish(enable_msg);
}

void Controller::publishCmdVel(double linear_vel, double angular_vel) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = linear_vel;
    cmd_vel.angular.z = angular_vel;
    cmd_vel_pub_.publish(cmd_vel);
}

void Controller::publishObsAvoidGoal() {
    const double SAFETY_MARGIN = 0.25;
    double goal_x, goal_y;
    
    Eigen::Vector2d local_offset;
    if (avoid_dir < 0) {
      local_offset = right_obs_pos_ + Eigen::Vector2d(SAFETY_MARGIN, -avoid_dir * SAFETY_MARGIN);
    } 
    else {
      local_offset = left_obs_pos_ + Eigen::Vector2d(SAFETY_MARGIN, -avoid_dir * SAFETY_MARGIN);  
    }
    
    Eigen::Rotation2D<double> rotation(cur_yaw_);
    Eigen::Vector2d global_offset = rotation * local_offset;
    goal_x = cur_pos_(0) + global_offset(0);
    goal_y = cur_pos_(1) + global_offset(1);

    /*ROS_INFO("cur pos: %.2f %.2f",cur_pos_(0),cur_pos_(1));
    ROS_INFO("goal: %.2f %.2f",goal_x,goal_y);*/
    double goal_yaw = cur_yaw_ - (-avoid_dir * (15.0 * M_PI / 180.0));
    goal_yaw = atan2(sin(goal_yaw), cos(goal_yaw));

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(goal_yaw), goal.pose.orientation);
    obs_goal_pub.publish(goal);
    obs_avoid_fsm.setGoal(goal);
}

void Controller::obsAvoid() {
    if(!pub_goal){
        publishObsAvoidGoal();
        pub_goal = true;
    }
    if (obs_avoid_fsm.returnState() == 4) { 
        ROS_INFO("Target reached, moving to lane recovery.");
        state = LANE_RECOVERY;
    }
}

void Controller::laneRecovery() {
    const double MAX_RECOV_TIME = 5.0;
    static ros::Time recov_start = ros::Time::now();
    double recov_rotation = 0.2 * -avoid_dir;
    //check ml confidence
    if (is_confident){
        ROS_INFO("Returning to lane.");
        state = LANE_FOLLOW;
    }
    else {
        //additional recovery
        //avoid dir * small angular velocity to move towards center?
        if ((ros::Time::now() - recov_start).toSec() > MAX_RECOV_TIME) {
            ROS_WARN("Lane recovery timeout. Rotating back to lane trajectory.");
            publishCmdVel(0.1, recov_rotation);
        }
    }
}

void Controller::emergencyStop() {
    ROS_WARN("EMERGENCY STOP TRIGGERED");
    ROS_WARN("Please Restart, lost track or critically close obstacle");
    publishCmdVel(0.0,0.0);
}
    

void Controller::controlLoop() {
    ros::Rate rate(10);
    while (ros::ok()) {
        switch (state) {
            case LANE_FOLLOW:
                toggleMLRoadFollow();
                break;
            case OBS_AVOID:
                toggleMLRoadFollow();
                obsAvoid();
                break;
            case LANE_RECOVERY:
                laneRecovery();
                break;
            case STOP:
                emergencyStop();
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle nh("~");
    Controller controller(nh);
    controller.controlLoop();
    return 0;
}