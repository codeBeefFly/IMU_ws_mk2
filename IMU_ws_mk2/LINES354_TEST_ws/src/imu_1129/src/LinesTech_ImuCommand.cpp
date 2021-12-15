//
// Created by ds18 on 12/2/21.
//

#include "LinesTech_ImuCommand.h"


bool LINESTECH::CImuCommand::display_decodeData(double *tmpBuffer) {
    for (int i = 0; i < DATA_LENGTH; i++) {
        std::cout << tmpBuffer[i] << std::endl;
    }
    return true;
}

bool LINESTECH::CImuCommand::display_tempData(double tmpBuffer) {
    std::cout << tmpBuffer << std::endl;
    return true;
}


bool LINESTECH::CImuCommand::display_Q4decodeData(double tmpBuffer[Q4DATA_LENGTH]) {
    for (int i = 0; i < Q4DATA_LENGTH; i++) {
        std::cout << tmpBuffer[i] << std::endl;
    }
    return true;
}

bool LINESTECH::CImuCommand::display_Imumsg(sensor_msgs::Imu imumsg) {
    std::cout << "print Q4 data!" << std::endl;
    std::cout << imumsg.orientation.x << std::endl;
    std::cout << imumsg.orientation.y << std::endl;
    std::cout << imumsg.orientation.z << std::endl;
    std::cout << imumsg.orientation.w << std::endl;

    std::cout << "print GRY data!" << std::endl;
    std::cout << imumsg.angular_velocity.x << std::endl;
    std::cout << imumsg.angular_velocity.y << std::endl;
    std::cout << imumsg.angular_velocity.z << std::endl;

    std::cout << "print ACC data!" << std::endl;
    std::cout << imumsg.linear_acceleration.x << std::endl;
    std::cout << imumsg.linear_acceleration.y << std::endl;
    std::cout << imumsg.linear_acceleration.z << std::endl;
}

bool LINESTECH::CImuCommand::decodeFrame(unsigned char tmpBuffer[READ_BUFFERSIZE]) {
//    ROS_INFO_STREAM("tmpBuffer[0]: " << tmpBuffer[0] << ", type: " << typeid(tmpBuffer[0]).name());  // sync. byte 1, 7FH
//    ROS_INFO_STREAM("tmpBuffer[1]: " << tmpBuffer[1] << ", type: " << typeid(tmpBuffer[1]).name());  // sync. byte 2, 80H

    // todo: this logic may discard later on
//    if (tmpBuffer[0] == DATA_FrameHead_1 && tmpBuffer[1] == DATA_FrameHead_2) {
//
//
//        ROS_INFO_STREAM("imu_raw: " << tmpBuffer);
//
//        total_count++;
//
//    } else {
//        std::cout << "数据帧头错误" << std::endl;
//
//        loss_count++;
//        total_count++;
//        ROS_INFO_STREAM("loss: " << loss_count << ", total: " << total_count << ", loss rate: " << float(loss_count) / float(total_count) * 100 << " %");
//
//        return false;
//    }

    // todo: need a new logic to ensure validate imu data input (done)
    for (int i = 0; i < READ_BUFFERSIZE - 23; ++i) {
        if (tmpBuffer[i] != DATA_FrameHead_1 || tmpBuffer[i + 1] != DATA_FrameHead_2) continue;

        // todo: store one imu data array of 23 bytes, we can store more than one as validate data buffer, but not now
        for (int j = 0; j < VALID_DATASIZE; ++j) {
            valid_data[j] = tmpBuffer[i + j];
//            ROS_INFO_STREAM("");
        }

        // todo: here is the logic for checksum, if check failed, re-loop
        unsigned char checkSum = valid_data[22];
        unsigned char sum = 0;
        unsigned char rev_sum = 0;

        for (int k = 2; k < VALID_DATASIZE - 1; ++k)
            sum = sum + valid_data[k];
        rev_sum = ~sum;
        total_count++;

        if (rev_sum == checkSum) {
//            ROS_INFO("imu data validated.");
            break;
        } else {
//            ROS_INFO("imu data invalid. re-loop.");
            loss_count++;
            return false;
        }

    }

//    ROS_INFO_STREAM("loss: " << loss_count << ", total: " << total_count << ", loss rate: " << float(loss_count) / float(total_count) * 100 << " %");

    /*
     * todo, decode raw data 23 bytes,
     * we have to use a local array variable to store the tmp data
     */

    auto *sensors = new double[10];  // acc, gyo, rpy, T

//    // acc
//    sensors[0] = (short) (tmpBuffer[2] + (tmpBuffer[3] << 8));       // x
//    sensors[1] = (short) (tmpBuffer[4] + (tmpBuffer[5] << 8));       // y
//    sensors[2] = (short) (tmpBuffer[6] + (tmpBuffer[7] << 8));       // z
//    // gyo
//    sensors[3] = (short) (tmpBuffer[8] + (tmpBuffer[9] << 8));       // x
//    sensors[4] = (short) (tmpBuffer[10] + (tmpBuffer[11] << 8));     // y
//    sensors[5] = (short) (tmpBuffer[12] + (tmpBuffer[13] << 8));     // z
//    // rpy
//    sensors[6] = (short) (tmpBuffer[14] + (tmpBuffer[15] << 8));     // x
//    sensors[7] = (short) (tmpBuffer[16] + (tmpBuffer[17] << 8));     // y
//    sensors[8] = (short) (tmpBuffer[18] + (tmpBuffer[19] << 8));     // z
//    // T
//    sensors[9] = (short) (tmpBuffer[20] + (tmpBuffer[21] << 8));     // T


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
//    ROS_INFO("==| ACC ----");
//    display_decodeData(ACC);
    imu_data.linear_acceleration.x = ACC[0];
    imu_data.linear_acceleration.y = ACC[1];
    imu_data.linear_acceleration.z = ACC[2];

    GYR[0] = sensors[3] * RATE_SCALE / SENSOR_SCALE;
    GYR[1] = sensors[4] * RATE_SCALE / SENSOR_SCALE;
    GYR[2] = sensors[5] * RATE_SCALE / SENSOR_SCALE;
//    ROS_INFO("==| GYRO ----");
//    display_decodeData(GYR);
    imu_data.angular_velocity.x = GYR[0];
    imu_data.angular_velocity.y = GYR[1];
    imu_data.angular_velocity.z = GYR[2];


    RPY[0] = sensors[6] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[1] = sensors[7] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[2] = sensors[8] * ANGLE_SCALE / SENSOR_SCALE;
//    ROS_INFO("==| RPY ----");
//    display_decodeData(RPY);
    imu_data.orientation.x = RPY[0];
    imu_data.orientation.y = RPY[1];
    imu_data.orientation.z = RPY[2];


    TP = sensors[9] * TEMP_SCALE / SENSOR_SCALE;
//    ROS_INFO("==| TEMP ----");
//    display_tempData(TP);

    return true;
}


bool LINESTECH::CImuCommand::cmdFrame(unsigned char imucmd) {
//    cmd_buffer[0] = CMD_FrameHead;
//    cmd_buffer[1] = imucmd;
//    cmd_buffer[2] = CMD_FrameHead + cmd_buffer[1];
//    return commd_buffer;

//    ser.write(cmd_buffer, cmd_num);  // why write?

    ser.read(r_buffer, READ_BUFFERSIZE);

    // use for debug
//    for (int i = 0; i < READ_BUFFERSIZE; i++) {
//        read_buffer[i] = (float) r_buffer[i];
//        ROS_INFO("[0x%02x]", r_buffer[i]);  // %02x means print at least 2 digits, prepend it with 0's if there's less. %x is for int
//    }
    //ROS_INFO("DECODE OUTPUT DATA!");
    decodeFrame(r_buffer);
    return true;
}


bool LINESTECH::CImuCommand::muliteCmdFrame(unsigned char imucmd1, unsigned char imucmd2, unsigned char imucmd3) {
    //第一个指令
    cmdFrame(imucmd1);
//    ROS_INFO("OUTPUT_Q4");   //如何输出宏定义的名字？？？？

    //第二个指令
    cmdFrame(imucmd2);
//    ROS_INFO("OUTPUT_GYR");

    //第三个指令
    cmdFrame(imucmd3);
//    ROS_INFO("OUTPUT_ACC");

    return true;
}


bool LINESTECH::CImuCommand::serialInit() {
    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial Port initialized");
        cmdFrame(CMD_OUTPUT_200HZ);
//        ser.write(cmd_buffer, cmd_num);  // why write?
    } else {
        return -1;
    }
    return true;
}

/**
 *
 * @return
 */
bool LINESTECH::CImuCommand::RUN() {

    ros::NodeHandle nh;

    // ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher msg_pub = nh.advertise<sensor_msgs::Imu>("imu_raw", 1000);

    //串口初始化
    serialInit();
    ros::Rate loop_rate(100);
    while (ros::ok()) {

        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_link";
        muliteCmdFrame(CMD_PULL_OUTPUT_Q4, CMD_PULL_OUTPUT_GYR, CMD_PULL_OUTPUT_ACC);
        cmdFrame(CMD_OUTPUT_200HZ);
//        display_Imumsg(imu_data);
        msg_pub.publish(imu_data);
        loop_rate.sleep();
    }

    return true;
}

