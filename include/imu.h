// 生成ros发布imu主题的代码
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

// 生成c++串口初始化和读取代码
#include <iostream>
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread.hpp"
#include <boost/bind.hpp>
#include <tf/tf.h>

using namespace std;

#define PI 3.1416

class imu
{
private:
    ros::NodeHandle n;
    ros::Publisher Pub_IMU_Raw_Data, Pub_IMU_AHRS_Data, Pub_GNSS_Data;
    sensor_msgs::Imu imu_raw_msg, imu_AHRS_msg;
    sensor_msgs::NavSatFix gnss_msg;

    int serial_fd = -1;
    string _IMUserial;
    int32_t _Baud;
    int _IMU_type;
    bool _IsPose_On;

    unsigned char buffer[16];
    short ax[6] = {0};
    short gx[6] = {0};
    short sAngle[6] = {0};
    short Magn[6] = {0};
    short Quat[8] = {0};
    short GNSS1[8] = {0};
    short GNSS2[8] = {0};
    short DOP[6] = {0};

    ros::Publisher Pub_Pos_Data;
    geometry_msgs::PoseStamped pose_data;

public:
    imu();
    ~imu();
    void process();
    int open_serial(string port, int baud_rate);
};