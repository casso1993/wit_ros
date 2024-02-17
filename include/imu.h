#include <ros/ros.h>
#include <serial/serial.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "boost/thread.hpp"
#include <boost/bind.hpp>
#include <assert.h>
#include <tf/tf.h>

#define PI 3.1416

using namespace std;
using namespace Eigen;

class imu
{
private:
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Publisher Pub_IMU_Raw_Data, Pub_IMU_AHRS_Data, Pub_GPS_Data, Pub_DOP_Data, Pub_Pos_Data;
    sensor_msgs::Imu imu_raw_data, imu_AHRS_data;
    sensor_msgs::NavSatFix gnss_data;
    geometry_msgs::PointStamped DOP_data;
    geometry_msgs::PoseStamped pose_data;
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(100);

public:
    imu();
    ~imu();
    void process();
    bool CheckSerial();
};
