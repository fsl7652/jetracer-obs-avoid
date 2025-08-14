#include <model_test/model_test.h>

ModelTest::ModelTest(ros::NodeHandle& nh) : nh_(nh)
{
    model_sub_ = nh_.subscribe("/steering_value", 10, &ModelTest::modelCallback, this);
    last_msg_time_ = ros::Time::now().toSec();
}
void ModelTest::modelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    double msg_time = ros::Time::now().toSec();
    model_data_ = *msg;
    times.push_back(msg_time - last_msg_time_);
    last_msg_time_ = msg_time;
    if (times.size() > 100) {
        ROS_WARN("Average time: %f", getAverageTime());
        times.clear();
    }
}

double ModelTest::getAverageTime()
{
    double sum = 0.0;
    for (double time : times) {
        sum += time;
    }
    return sum / times.size();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "model_test_node");
    ros::NodeHandle nh;
    ModelTest model_test(nh);
    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}