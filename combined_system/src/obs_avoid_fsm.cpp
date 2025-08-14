#include <plan_manage/obs_avoid_fsm.h>
#include <chrono>

void ObsAvoidFSM::init(ros::NodeHandle &nh){
    nh_ = nh;
    have_goal = false;
    collision = false;

    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    nh_.param("vehicle/car_d_cr", car_d_cr_, 1.015);
    map.reset(new MappingProcess);
    map->init(nh);

    planner.reset(new TrajPlanner);
    planner->setMap(map);
    planner->init(nh);
    
    odom_sub   = nh_.subscribe("/odom", 100, &ObsAvoidFSM::OdomCallback, this);

    main_timer = nh_.createTimer(ros::Duration(0.02), &ObsAvoidFSM::FSMCallback, this);
}

void ObsAvoidFSM::OdomCallback(const nav_msgs::Odometry& msg)
{
    try {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));

        Eigen::Vector3d map_pos(
            transformStamped.transform.translation.x,
            transformStamped.transform.translation.y,
            transformStamped.transform.translation.z
        );

        Eigen::Quaterniond quat(
            transformStamped.transform.rotation.w,
            transformStamped.transform.rotation.x,
            transformStamped.transform.rotation.y,
            transformStamped.transform.rotation.z
        );

        cur_pos = map_pos.head<2>();
        cur_yaw = tf::getYaw(transformStamped.transform.rotation);

    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Lookup failed in ObsAvoidFSM: %s", ex.what());
        return;
    }
    cur_vel = msg.twist.twist.linear.x;
    if (state == EXECUTE) {
        double time_now = ros::Time::now().toSec();
        collision = planner->checkCollisionWithObs(time_now);
    }
}

void ObsAvoidFSM::setGoal(const geometry_msgs::PoseStamped msg)
{
    std::cout << "Triggered Obstacle Avoidance" << std::endl;
    goal << msg.pose.position.x, msg.pose.position.y, 
               tf::getYaw(msg.pose.orientation), 1.0e-2;
    std::cout<<"Goal: "<<goal.transpose()<<std::endl;
    have_goal = true;
}

void ObsAvoidFSM::FSMCallback(const ros::TimerEvent& event)
{
    main_timer.stop();
    static bool target_wait_message_shown = false; 

    if (!have_goal) {
         if (!target_wait_message_shown) 
            {
                cout << "Waiting for goal." << endl;
                target_wait_message_shown = true;
            }
    }
    else {
        target_wait_message_shown = false;
        
    }

    switch (state){
        case WAIT:
        {
            if(!have_goal){
                goto loop;
            }
            else {
                state = START;
            }
        break;
        }
        case START:
        {
            pos << cur_pos(0), cur_pos(1) , cur_yaw, cur_vel;
            double start_time = ros::Time::now().toSec();
            double budget_time = start_time + BUDGET;
            planner->setInitStateAndInput(pos, budget_time);
            planner->setGoal(goal);
            planner->getKinoPath(goal,true);
            planner->displayKinoPath(planner->display_kino_path());
            planner->RunAvoidance();
            double planning_time = ros::Time::now().toSec() - start_time;
            if (BUDGET > planning_time)
            {
                ros::Duration(BUDGET - planning_time).sleep();
            }
            else
            {
                    ROS_ERROR("Out of time budget!");
            }
            std::cout << "Planning time: " << planning_time << " seconds" << std::endl;
            planner->displayTraj(planner->trajectory());
            state = EXECUTE;
            break;
        }
        case EXECUTE:
        {
            publishVel();
            ros::Time t_now = ros::Time::now();
            

            if (((cur_pos - pos.head(2)).norm() > 3.0 || (t_now.toSec() - budget_time) > 5) && (cur_pos - goal.head(2)).norm() > 5.0)
            {
                std::cout << "Trajectory stale & too far from goal, replanning" << std::endl;
                state = REPLAN;
                break;
            }

            if ((collision ) && t_now.toSec() - budget_time > BUDGET ) 
            {
                std::cout << "Collision detected, replanning" << std::endl;
                state = REPLAN;
                break;
            }
            // reach end
            if ((cur_pos - goal.head(2)).norm() < 0.15 && abs(cur_yaw - goal(2) < 0.05 && abs(cur_vel) < 0.2))
            {
                state = AT_GOAL;
                std::cout << "Reached goal" << std::endl;
                have_goal = false;
                goto loop;
            }
            if (traj_completed) {
                state = REPLAN;
                break;
            }
            break;
        }
        case REPLAN:
        {
                double replan_start = ros::Time::now().toSec();
                double budget_time = replan_start + BUDGET;
                pos << cur_pos(0), cur_pos(1), cur_yaw, cur_vel;
                planner->setInitStateAndInput(pos, budget_time);
                planner->setGoal(goal);
    
                ros::Time t1 = ros::Time::now();
                bool kino_success = planner->getKinoPath(goal, false);
                if (!kino_success)
                {
                    std::cout << "KinoPath failed. Waiting for recovery..." << std::endl;
                    ros::Duration(0.2).sleep();
                    break;
                }
    
                planner->displayKinoPath(planner->display_kino_path());
    
                bool opt_success = planner->RunAvoidance();
                if (!opt_success)
                {
                    std::cout << "Optimization failed. Waiting for recovery..." << std::endl;
                    ros::Duration(0.1).sleep();
                    break;
                }
    
                ros::Time t2 = ros::Time::now();
                double planning_time = (t2 - t1).toSec();
    
                if (planning_time < BUDGET)
                {
                    ros::Duration(BUDGET - planning_time).sleep();
                }
                else
                {
                    ROS_WARN("Planning exceeded budget time!");
                }
    
                planner->displayTraj(planner->trajectory());
    
                state = EXECUTE;
                break;
            }
            case AT_GOAL:
            {
                if (!have_goal){
                    goto loop;
                }
                else
                {
                    state = START;
                }   
                break;
            }
    }
loop:;
    main_timer.start();
}

int ObsAvoidFSM::returnState() const
{
    return state;
}

void ObsAvoidFSM::publishVel(){


    geometry_msgs::Twist cmd_vel_msg;
    double linear_vel, angular_vel;
    bool is_completed = planner->computeVel(linear_vel, angular_vel);

    cmd_vel_msg.linear.x = linear_vel;
    cmd_vel_msg.linear.y = 0.0;
     cmd_vel_msg.linear.z = 0.0;

    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = angular_vel;

    cmd_vel_pub.publish(cmd_vel_msg);
    traj_completed = is_completed;
}
