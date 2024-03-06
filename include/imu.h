#include <ros/ros.h>
#include <serial/serial.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <assert.h>
#include <tf/tf.h>

#define PI 3.1416

using namespace std;

class imu
{
private:
    ros::NodeHandle n;
    ros::Publisher Pub_IMU_Raw_Data, Pub_IMU_AHRS_Data, Pub_GPS_Data, Pub_Pos_Data;
    sensor_msgs::Imu imu_raw_data, imu_AHRS_data;
    sensor_msgs::NavSatFix gnss_data;
    geometry_msgs::PoseStamped pose_data;
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);

    double a[3];
    double w[3];
    double angle[3];
    double longitude, latitude, altitude;
    double GPS_Yaw, GPSV;
    double q[4];
    double satellite;
    double DOP[3];
    geometry_msgs::Quaternion msg_q;
    geometry_msgs::Quaternion msg_AHRS_q;

public:
    imu();
    ~imu();
    void process();
};
