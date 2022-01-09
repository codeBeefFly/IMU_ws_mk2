//
// Created by ds18 on 1/8/22.
//

#include "ros/ros.h"
#include "rosbag/bag.h"
#include "std_msgs/String.h"


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "demo_chp05_bag_write");
    ros::NodeHandle nh;

    //创建bag对象
    rosbag::Bag bag;

    //打开
    bag.open("/home/ds18/catkin_x2/docs/test.bag", rosbag::BagMode::Write);

    //写
    std_msgs::String msg;
    msg.data = "hello world";
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);
    bag.write("/chatter", ros::Time::now(), msg);

    //关闭
    bag.close();

    return 0;
}
