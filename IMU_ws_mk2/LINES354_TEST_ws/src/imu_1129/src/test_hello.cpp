//
// Created by ds18 on 11/29/21.
//

#include <iostream>
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hello");

    ros::NodeHandle node;

    std::cout << "hello ros!" << std::endl;

    return 0;
}

