//
// Created by ds18 on 12/25/21.
//

/*
 * 创建第二只小乌龟
 */
#include "ros/ros.h"
#include "turtlesim/Spawn.h"

int main(int argc, char *argv[]) {

    setlocale(LC_ALL, "");

    //执行初始化
    ros::init(argc, argv, "demo_09_create_turtle_node");
    //创建节点
    ros::NodeHandle nh;

    //创建服务客户端
    ros::ServiceClient client = nh.serviceClient<turtlesim::Spawn>("/spawn");

    ros::service::waitForService("/spawn");
    turtlesim::Spawn spawn;
    spawn.request.name = "turtle2";
    spawn.request.x = 1.0;
    spawn.request.y = 2.0;
    spawn.request.theta = 3.12415926;
    bool flag = client.call(spawn);
    if (flag) {
        ROS_INFO("乌龟 %s 创建成功!", spawn.response.name.c_str());
    } else {
        ROS_INFO("乌龟 %s 创建失败!", spawn.response.name.c_str());
    }

    ros::spin();

    return 0;
}

