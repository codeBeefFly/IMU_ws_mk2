//
// Created by ds18 on 12/16/21.
//

#ifndef SRC_LINESTECHIMU_H
#define SRC_LINESTECHIMU_H

////////////////////////////////////////

#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"

// marco constant
#define DATA_FrameHead_1              (unsigned char)(127)
#define DATA_FrameHead_2              (unsigned char)(128)
#define READ_BUFFER_SIZE                               115
#define VALID_DATA_SIZE                                 23
#define DATA_LENGTH                                      3

// 定义比例系数
#define ACCEL_SCALE                                     20
#define RATE_SCALE                                    1260
#define ANGLE_SCALE                                    360
#define SENSOR_SCALE                                 65536
#define TEMP_SCALE                                     200


namespace LinesTech {

class ImuDriver {
public:

    ImuDriver() {};

    ~ImuDriver() {};

    /**
     * dummy function for test usage
     */
    void dummy();

    /**
     * read raw imu data from serial port
     */
    void readFromSerialPort();

    /**
     * process imu raw data (checksum, processRawData)
     * @param buffer data buffer of 23 bytes
     * @return true if decode complete, false otherwise
     */
    bool processRawData(unsigned char tmpBuffer[READ_BUFFER_SIZE]);

    /**
     * display data of 1 byte
     * @param tmpBuffer data of 1 byte
     * @return true if displayed, false otherwise
     */
    void displayTempData(double tmpData, const std::string &name);

    /**
     * display data of 3 bytes
     * @param tmpData data of 3 bytes
     * @return true if displayed, false otherwise
     */
    void displayDecodeData(double tmpData[DATA_LENGTH], const std::string &name);

    /**
     * serial port initialization
     * @return true if initialized
     */
    bool serialInit();

    /**
     * Imu driver main entry point
     * @return true if running ok
     */
    bool runImu();

    /**
     * do checksum validation
     * @param data imu valid data of 23 bytes
     * @return true if data validated, false otherwise
     */
    bool validateChecksum(unsigned char data[23]);

    /**
     * decode valid 23 bytes raw data and assign them to imu msg structure
     * @param data valid 23 bytes raw data
     * @return true if decode complete, false otherwise
     */
    bool decodeRawData(const unsigned char data[23]);

private:

    serial::Serial ser;
    sensor_msgs::Imu imu_data;

    unsigned char r_buffer[READ_BUFFER_SIZE];
    unsigned char valid_data[VALID_DATA_SIZE];
    double GYR[DATA_LENGTH];
    double ACC[DATA_LENGTH];

    double RPY[DATA_LENGTH];  // 0-R, 1-P, 2-Y
    double TP;
    int total_count = 0;
    int loss_count = 0;
};

}   // end of ns LinesTech


////////////////////////////////////////

#endif //SRC_LINESTECHIMU_H
