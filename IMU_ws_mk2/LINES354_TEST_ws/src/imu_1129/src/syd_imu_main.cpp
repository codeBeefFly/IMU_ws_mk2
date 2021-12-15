//
// Created by ds18 on 12/2/21.
//

// Step 1:  Include Library Headers:
#include "EasyObjectDictionary.h"
#include "EasyProfile.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include <boost/asio.hpp>         // 包含boost库函数
#include <boost/bind.hpp>

using namespace boost::asio;

int main(int argc, char **argv) {
    // Step 2: Initialization:
    EasyProfile_C_Interface_Init();

    ros::init(argc, argv, "imu");
    ros::NodeHandle n;

    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);

    io_service io;
    serial_port sp(io, "/dev/ttyUSB0");                                     // 定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));                        // 波特率
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));    // 流量控制
    sp.set_option(serial_port::parity(serial_port::parity::none));                // 奇偶校验
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));           // 停止位
    sp.set_option(serial_port::character_size(8));                          // 数据位

    ros::Rate loop_rate(50);
    while (ros::ok()) {
        // Step 3 and Step 4 are optional, only if you want to use the request-response communication pattern
        // Step 3: Request quaternion Data from TransdcuerM
        uint16 toId = 309; // Node ID
        char *txData;
        int txSize;
        if (EP_SUCC_ == EasyProfile_C_Interface_TX_Request(toId, EP_CMD_Q_S1_E_, &txData, &txSize)) {
            write(sp, buffer(txData, txSize));    // Step 4:  Send the request via Serial Port.
        }

        char rxData[100];
        boost::system::error_code err;

        sp.read_some(buffer(rxData, 50), err);  // Read from serial port buffer
        if (err) {
            ROS_INFO("Serial port read_some Error!");
            return -1;
        }

        Ep_Header header;                           // Then let the EasyProfile do the rest such as data assembling and checksum verification.
        if (EP_SUCC_ == EasyProfile_C_Interface_RX((char *) rxData, 50, &header)) {
            // Quanternion received
            unsigned int timeStamp = ep_Q_s1_e.timeStamp;
            float q1 = ep_Q_s1_e.q[0];              // Note 1, ep_Q_s1_e is defined in the EasyProfile library as a global variable
            float q2 = ep_Q_s1_e.q[1];              // Note 2, for the units and meaning of each value, refer to EasyObjectDictionary.h
            float q3 = ep_Q_s1_e.q[2];
            float q4 = ep_Q_s1_e.q[3];
            ROS_INFO("Q: %f %f %f %f\n", q1, q2, q3, q4);

            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";
            imu_data.orientation.x = q3;
            imu_data.orientation.y = -q2;
            imu_data.orientation.z = -q1;
            imu_data.orientation.w = q4;

            IMU_pub.publish(imu_data);
        }

        io.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}