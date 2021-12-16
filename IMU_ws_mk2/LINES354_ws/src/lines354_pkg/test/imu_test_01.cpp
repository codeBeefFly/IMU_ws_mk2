#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"

#define DATA_FrameHead_1              (unsigned char)(127)
#define DATA_FrameHead_2              (unsigned char)(128)
#define READ_BUFFER_SIZE                               115
#define VALID_DATA_SIZE                                 23
#define DATA_LENGTH                                      3

//定义比例系数
#define ACCEL_SCALE                                     20
#define RATE_SCALE                                    1260
#define ANGLE_SCALE                                    360
#define SENSOR_SCALE                                 65536
#define TEMP_SCALE                                     200

unsigned char r_buffer[READ_BUFFER_SIZE];
unsigned char valid_data[VALID_DATA_SIZE];
double GYR[DATA_LENGTH];
double ACC[DATA_LENGTH];
double RPY[DATA_LENGTH];  // 0-R, 1-P, 2-Y

double TP;

int total_count = 0;
int loss_count = 0;

void cmdFrame(serial::Serial &serial1, sensor_msgs::Imu &imu_data1);

bool decodeFrame(unsigned char *buffer, sensor_msgs::Imu &imu_data1);

bool display_tempData(double tmpBuffer);

bool display_decodeData(double *tmpBuffer);

//serial::Serial ser;               // do not use it here
//sensor_msgs::Imu imu_data;        // do not use it here


/******************************************************
 * testing code function main
 * @param argc
 * @param argv
 * @return
 *****************************************************/
int main(int argc, char **argv) {
    // 1. ros init, node_name: imu_test_01_node
    ros::init(argc, argv, "imu_test_01_node");

    // 2. ros node handler
    ros::NodeHandle nh;

    // 3. publisher, topic_name: imu_msg
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu_msg", 100);

    // 4. init. serial port
    serial::Serial ser;

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e) {
        ROS_ERROR("Unable to open port!...");
        return -1;
    }

    // 5. read from serial port
    if (ser.isOpen()) {
        ROS_INFO("Serial Port initialized...");
//        cmdFrame(ser);    //todo: read imu raw data
    } else {
        return -1;
    }

    // 6. continue reading from serial port todo
    sensor_msgs::Imu imu_data;

    ros::Rate loop_rate(100);
    while (ros::ok()) {

        imu_data.header.stamp = ros::Time::now();
        imu_data.header.frame_id = "imu_link";
        cmdFrame(ser, imu_data);
        imu_pub.publish(imu_data);
        loop_rate.sleep();
    }

    // 5. end of imu commu.
    ROS_INFO("IMU commu. exited...");

    return 0;
}

void cmdFrame(serial::Serial &s, sensor_msgs::Imu &imu) {

    s.read(r_buffer, READ_BUFFER_SIZE);
    decodeFrame(r_buffer, imu);

}

bool decodeFrame(unsigned char tmpBuffer[READ_BUFFER_SIZE], sensor_msgs::Imu &imu_data) {

    // todo: need a new logic to ensure validate imu data input (done)
    for (int i = 0; i < READ_BUFFER_SIZE - 23; ++i) {
        if (tmpBuffer[i] != DATA_FrameHead_1 || tmpBuffer[i + 1] != DATA_FrameHead_2) continue;

        // todo: store one imu data array of 23 bytes, we can store more than one as validate data buffer, but not now
        for (int j = 0; j < VALID_DATA_SIZE; ++j) {
            valid_data[j] = tmpBuffer[i + j];
        }

        // todo: here is the logic for checksum, if check failed, re-loop
        unsigned char checkSum = valid_data[22];
        unsigned char sum = 0;
        unsigned char rev_sum = 0;

        for (int k = 2; k < VALID_DATA_SIZE - 1; ++k)
            sum = sum + valid_data[k];
        rev_sum = ~sum;
        total_count++;

        if (rev_sum == checkSum) {
            ROS_INFO("imu data validated.");
            break;
        } else {
            ROS_INFO("imu data invalid. re-loop.");
            loss_count++;
            return false;
        }
    }

    ROS_INFO_STREAM("loss: " << loss_count << ", total: " << total_count << ", loss rate: "
                             << float(loss_count) / float(total_count) * 100 << " %");

    /*
     * todo, decode raw data 23 bytes,
     * we have to use a local array variable to store the tmp data
     */

    auto *sensors = new double[10];  // acc, gyo, rpy, T
//    sensor_msgs::Imu imu_data;

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
    ROS_INFO("==| ACC ----");
    display_decodeData(ACC);
    imu_data.linear_acceleration.x = ACC[0];
    imu_data.linear_acceleration.y = ACC[1];
    imu_data.linear_acceleration.z = ACC[2];

    GYR[0] = sensors[3] * RATE_SCALE / SENSOR_SCALE;
    GYR[1] = sensors[4] * RATE_SCALE / SENSOR_SCALE;
    GYR[2] = sensors[5] * RATE_SCALE / SENSOR_SCALE;
    ROS_INFO("==| GYRO ----");
    display_decodeData(GYR);
    imu_data.angular_velocity.x = GYR[0];
    imu_data.angular_velocity.y = GYR[1];
    imu_data.angular_velocity.z = GYR[2];

    RPY[0] = sensors[6] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[1] = sensors[7] * ANGLE_SCALE / SENSOR_SCALE;
    RPY[2] = sensors[8] * ANGLE_SCALE / SENSOR_SCALE;
    ROS_INFO("==| RPY ----");
    display_decodeData(RPY);
    imu_data.orientation.x = RPY[0];
    imu_data.orientation.y = RPY[1];
    imu_data.orientation.z = RPY[2];

    TP = sensors[9] * TEMP_SCALE / SENSOR_SCALE;
    ROS_INFO("==| TEMP ----");
    display_tempData(TP);

    return true;
}


bool display_tempData(double tmpBuffer) {
    std::cout << tmpBuffer << std::endl;
    return true;
}

bool display_decodeData(double *tmpBuffer) {
    for (int i = 0; i < DATA_LENGTH; i++) {
        std::cout << tmpBuffer[i] << std::endl;
    }
    return true;
}