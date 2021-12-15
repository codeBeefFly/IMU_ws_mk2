//
// Created by ds18 on 11/30/21.
//

#include "test_ImuCommand.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_ImuCommand_node");
    IPSG::CImuCommand ImuCmd;
    ImuCmd.RUN();

    return true;
}



