//
// Created by ds18 on 12/2/21.
//

#include "LinesTech_ImuCommand.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "LinesTech_node");
    LINESTECH::CImuCommand ImuCmd;
    ImuCmd.RUN();

    return true;
}
