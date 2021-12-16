//
// Created by ds18 on 12/16/21.
//

//#include <ros/init.h>
#include "lines354_pkg/linesTechImu.h"

int main(int argc, char ** argv){
    ros::init(argc, argv, "linesTech_imu_node");
    LinesTech::ImuDriver imu_driver;
    imu_driver.runImu();

    return 0;
}