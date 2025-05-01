#ifndef MODEL_TEST_H
#define MODEL_TEST_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <list>
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>

class ModelTest
{
public:
    ModelTest(ros::NodeHandle& nh);
private:
    ros::NodeHandle& nh_;
    void modelCallback(const std_msgs::Float32::ConstPtr& msg);
    ros::Subscriber model_sub_;
    std_msgs::Float32 model_data_;
    double msg_time_;
    double last_msg_time_;
    std::list<double> times;
    double getAverageTime();
};

#endif