//
// Created by ds18 on 12/16/21.
//

//#include "ros/ros.h"
#include "lines354_pkg/linesTechImu.h"

namespace LinesTech {

void ImuDriver::dummy() {
    ROS_INFO("dummy ros...");
}

bool ImuDriver::runImu() {

    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_msg", 100);

    if(!serialInit())   return false;

    ros::Rate loop_rate(100);
    while (ros::ok()) {
        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_link";
        readFromSerialPort();
        imu_pub.publish(imu_data);
        loop_rate.sleep();
    }

    return false;
}

bool ImuDriver::serialInit() {
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port!...");
        return false;
    }

    if (ser.isOpen()) {
        ROS_INFO("Serial Port initialized...");
    } else {
        return false;
    }

    return true;
}

void ImuDriver::readFromSerialPort() {
    ser.read(r_buffer, READ_BUFFER_SIZE);
    processRawData(r_buffer);
}

bool ImuDriver::processRawData(unsigned char *tmpBuffer) {

    // store valid 23 bytes data, and calculate checksum
    for (int i = 0; i < READ_BUFFER_SIZE - 23; ++i) {

        if (tmpBuffer[i] != DATA_FrameHead_1 || tmpBuffer[i + 1] != DATA_FrameHead_2)
            continue;

        // store one imu data array of 23 bytes
        for (int j = 0; j < VALID_DATA_SIZE; ++j) {
            valid_data[j] = tmpBuffer[i + j];
        }

        // validate checksum, if failed, then re-loop
        bool rev = validateChecksum(valid_data);
        if (rev)    break;
    }
    ROS_DEBUG("loss: %d, total: %d, loss rate: %f%%", loss_count, total_count,
              (float(loss_count) / float(total_count)) * 100);

    // decode valid raw data
    if (!decodeRawData(valid_data))
        ROS_ERROR("Data decoding failed!...");

    return false;
}

bool ImuDriver::validateChecksum(unsigned char *data) {
    unsigned char checkSum = valid_data[22];
    unsigned char sum = 0;
    unsigned char rev_sum = 0;

    for (int k = 2; k < VALID_DATA_SIZE - 1; ++k)
        sum = sum + valid_data[k];
    rev_sum = ~sum;
    total_count++;

    if (rev_sum == checkSum) {
        ROS_INFO("imu data validated.");
        return true;
    } else {
        ROS_INFO("imu data invalid. re-loop.");
        loss_count++;
        return false;
    }
}

bool ImuDriver::decodeRawData(const unsigned char *data) {

    int len1 = sizeof(data);
    int len2 = sizeof(data[0]);
    int len = len1 / len2;
    if (len != 23) {
        ROS_ERROR("Valid data length less than 23 bytes!...");
        return false;
    }

    auto *sensors = new double[10];  // acc, gyo, rpy, T

    // acc
    sensors[0] = (int16_t) (valid_data[2] + (valid_data[3] << 8));       // x
    sensors[1] = (int16_t) (valid_data[4] + (valid_data[5] << 8));       // y
    sensors[2] = (int16_t) (valid_data[6] + (valid_data[7] << 8));       // z
    // gyo
    sensors[3] = (int16_t) (valid_data[8] + (valid_data[9] << 8));       // x
    sensors[4] = (int16_t) (valid_data[10] + (valid_data[11] << 8));     // y
    sensors[5] = (int16_t) (valid_data[12] + (valid_data[13] << 8));     // z
    // rpy
    sensors[6] = (int16_t) (valid_data[14] + (valid_data[15] << 8));     // x
    sensors[7] = (int16_t) (valid_data[16] + (valid_data[17] << 8));     // y
    sensors[8] = (int16_t) (valid_data[18] + (valid_data[19] << 8));     // z
    // T
    sensors[9] = (int16_t) (valid_data[20] + (valid_data[21] << 8));     // T

    ACC[0] = sensors[0] * ACCEL_SCALE / SENSOR_SCALE;
    ACC[1] = sensors[1] * ACCEL_SCALE / SENSOR_SCALE;
    ACC[2] = sensors[2] * ACCEL_SCALE / SENSOR_SCALE;
    imu_data.linear_acceleration.x = ACC[0];
    imu_data.linear_acceleration.y = ACC[1];
    imu_data.linear_acceleration.z = ACC[2];

    GYR[0] = sensors[3] * RATE_SCALE / SENSOR_SCALE;
    GYR[1] = sensors[4] * RATE_SCALE / SENSOR_SCALE;
    GYR[2] = sensors[5] * RATE_SCALE / SENSOR_SCALE;
    imu_data.angular_velocity.x = GYR[0];
    imu_data.angular_velocity.y = GYR[1];
    imu_data.angular_velocity.z = GYR[2];

    RPY[0] = sensors[6] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[1] = sensors[7] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[2] = sensors[8] * ANGLE_SCALE / SENSOR_SCALE;
    imu_data.orientation.x = RPY[0];
    imu_data.orientation.y = RPY[1];
    imu_data.orientation.z = RPY[2];

    TP = sensors[9] * TEMP_SCALE / SENSOR_SCALE;

    displayDecodeData(ACC, "ACC");
    displayDecodeData(GYR, "GYRO");
    displayDecodeData(RPY, "RPY");
    displayTempData(TP, "TEMPERATURE");

    return true;
}

void ImuDriver::displayTempData(double tmpData, const std::string &name) {
    ROS_DEBUG("%s:\n-----: %f", name.c_str(), tmpData);
}

void ImuDriver::displayDecodeData(double *tmpData, const std::string &name) {
    ROS_DEBUG("%s:", name.c_str());
    for (int i = 0; i < DATA_LENGTH; i++) {
        ROS_DEBUG("----: %f", tmpData[i]);
    }
}


}   // end of ns LinesTech
