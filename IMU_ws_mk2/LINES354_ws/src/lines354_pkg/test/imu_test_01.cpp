#include "ros/ros.h"



int main(int argc, char ** argv)
{
    // 1. ros init
    ros::init(argc, argv, "imu_test_01");

    // 2. ros node handler
    ros::NodeHandle nh;

    ROS_INFO("hello ros again!");


    return 0;
}