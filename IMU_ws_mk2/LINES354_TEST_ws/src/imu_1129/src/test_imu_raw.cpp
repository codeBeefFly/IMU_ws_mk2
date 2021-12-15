//
// Created by ds18 on 11/29/21.
// link: ROS从入门到精通系列（十六）-- IMU in ROS
//       https://blog.csdn.net/hhaowang/article/details/104961725
//

#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include <sstream>

/*!
 * imu callback function
 * @param imu_raw : imu msg pointer
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr& imu_raw)
{
    ROS_INFO_STREAM("Imu raw data publish working: " << imu_raw->header.stamp);
    ROS_INFO_STREAM("Data samples orientation    : " << imu_raw->orientation);

}


/**
 * test function main
 * @param argc
 * @param argv
 * @return 0
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "imu_raw_pub");  // node name

    ros::NodeHandle n;

    ros::Publisher imu_raw_pub = n.advertise<sensor_msgs::Imu>("imu_data", 10, imu_callback);  // ros_topic

    ros::Rate loop_rate(1);  // frequency 1 hz

    while(ros::ok())
    {
        sensor_msgs::Imu imu_raw;

        imu_raw.header.stamp = ros::Time::now();
        imu_raw.header.frame_id = "imu_link";

        /*
         * quaternion data
         */
        imu_raw.orientation.x = 0;
        imu_raw.orientation.y = -1;
        imu_raw.orientation.z = -5;
        imu_raw.orientation.w = 6;

        /*
         * linear acceleration
         */
        imu_raw.linear_acceleration.x = 0.01;
        imu_raw.linear_acceleration.y = 0.02;
        imu_raw.linear_acceleration.z = 0.03;

        /*
         * angular velocity
         */
        imu_raw.angular_velocity.x = 0.05;
        imu_raw.angular_velocity.y = 0.06;
        imu_raw.angular_velocity.z = 0.07;

        /*
         * publish topic
         */
        imu_raw_pub.publish(imu_raw);  // publish msgs


    }

    ros::spinOnce();
    loop_rate.sleep();

    return 0;
}

