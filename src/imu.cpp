#include "imu.h"

string _IMUserial;
int32_t _Baud;

unique_ptr<imu> imu_ptr;

void imu::process()
{
    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return;
    }

    assert(sp.isOpen());

    while (ros::ok())
    {
        // 获取缓冲区内的字节数
        size_t n = sp.available();
        if (n != 0)
        {
            uint8_t buffer[1024];
            // 读出数据
            n = sp.read(buffer, n);

            for (int i = 0; i < n; i++)
            {
                if (buffer[i] == 0X55) // IMU地址
                {
                    if (buffer[i + 1] == 0x51) // 加速度帧
                    {
                        a[0] = (short)((short)(buffer[i + 3] << 8) | buffer[i + 2]) / 32768.0 * 16 * 9.8;
                        a[1] = (short)((short)(buffer[i + 5] << 8) | buffer[i + 4]) / 32768.0 * 16 * 9.8;
                        a[2] = (short)((short)(buffer[i + 7] << 8) | buffer[i + 6]) / 32768.0 * 16 * 9.8;
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x52) // 角速度帧
                    {
                        w[0] = (short)((short)(buffer[i + 3] << 8) | buffer[i + 2]) / 32768.0 * 2000.0;
                        w[1] = (short)((short)(buffer[i + 5] << 8) | buffer[i + 4]) / 32768.0 * 2000.0;
                        w[2] = (short)((short)(buffer[i + 7] << 8) | buffer[i + 6]) / 32768.0 * 2000.0;
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x53) // 角度帧
                    {
                        angle[0] = (short)((short)(buffer[i + 3] << 8) | buffer[i + 2]) / 32768.0 * PI;
                        angle[1] = (short)((short)(buffer[i + 5] << 8) | buffer[i + 4]) / 32768.0 * PI;
                        angle[2] = (short)((short)(buffer[i + 7] << 8) | buffer[i + 6]) / 32768.0 * PI;
                        msg_q = tf::createQuaternionMsgFromRollPitchYaw(angle[0], angle[1], angle[2]);
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x57) // GPS帧
                    {
                        unsigned long int temp = (buffer[i + 5] << 24 | buffer[i + 4] << 16 | buffer[i + 3] << 8 | buffer[i + 2]);
                        longitude = temp / 10000000 + (double)(temp % 10000000) / 10000000 / 0.6;
                        temp = (buffer[i + 9] << 24 | buffer[i + 8] << 16 | buffer[i + 7] << 8 | buffer[i + 6]);
                        latitude = temp / 10000000 + (double)(temp % 10000000) / 10000000 / 0.6;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x58) // GPS帧
                    {
                        altitude = (short)((short)buffer[i + 3] << 8 | buffer[i + 2]) / 10.0;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x59) // 四元数
                    {
                        q[0] = (short)((short)buffer[i + 3] << 8 | buffer[i + 2]) / 32768.0;
                        q[1] = (short)((short)buffer[i + 5] << 8 | buffer[i + 4]) / 32768.0;
                        q[2] = (short)((short)buffer[i + 7] << 8 | buffer[i + 6]) / 32768.0;
                        q[3] = (short)((short)buffer[i + 9] << 8 | buffer[i + 8]) / 32768.0;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x5A) // GPS精度
                    {
                        satellite = (short)((short)buffer[i + 3] << 8 | buffer[i + 2]);
                        DOP[0] = (short)((short)buffer[i + 5] << 8 | buffer[i + 4]) / 100.0;
                        DOP[1] = (short)((short)buffer[i + 7] << 8 | buffer[i + 6]) / 100.0;
                        DOP[2] = (short)((short)buffer[i + 9] << 8 | buffer[i + 8]) / 100.0;
                        i += 11;
                    }

                    imu_raw_data.header.stamp = ros::Time::now();
                    imu_raw_data.header.frame_id = "IMU";
                    imu_raw_data.linear_acceleration.x = a[0];
                    imu_raw_data.linear_acceleration.y = a[1];
                    imu_raw_data.linear_acceleration.z = a[2];
                    imu_raw_data.angular_velocity.x = w[0];
                    imu_raw_data.angular_velocity.y = w[1];
                    imu_raw_data.angular_velocity.z = w[2];
                    imu_raw_data.orientation = msg_q;
                    Pub_IMU_Raw_Data.publish(imu_raw_data);

                    imu_AHRS_data.header.stamp = ros::Time::now();
                    imu_AHRS_data.header.frame_id = "IMU";
                    imu_AHRS_data.linear_acceleration.x = a[0];
                    imu_AHRS_data.linear_acceleration.y = a[1];
                    imu_AHRS_data.linear_acceleration.z = a[2];
                    imu_AHRS_data.angular_velocity.x = w[0];
                    imu_AHRS_data.angular_velocity.y = w[1];
                    imu_AHRS_data.angular_velocity.z = w[2];
                    imu_AHRS_data.orientation.w = q[0];
                    imu_AHRS_data.orientation.x = q[1];
                    imu_AHRS_data.orientation.y = q[2];
                    imu_AHRS_data.orientation.z = q[3];
                    Pub_IMU_AHRS_Data.publish(imu_AHRS_data);

                    ros::Time time = ros::Time::now();
                    gnss_data.header.stamp = time;
                    gnss_data.header.frame_id = "GNSS";
                    gnss_data.latitude = latitude;
                    gnss_data.longitude = longitude;
                    gnss_data.altitude = altitude;
                    gnss_data.position_covariance[0] = DOP[0];
                    gnss_data.position_covariance[4] = DOP[1];
                    gnss_data.position_covariance[8] = DOP[2];
                    Pub_GPS_Data.publish(gnss_data);
                }
            }
        }
    }

    // 关闭串口
    sp.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Integrated_Navigation");
    imu_ptr.reset(new imu());
    imu_ptr->process();
    return 0;
}

imu::imu()
{
    // 创建imu数据话题发布
    ros::NodeHandle nh("~");
    nh.param<std::string>("IMUserial", _IMUserial, "/dev/ttyUSB0");
    nh.param<std::int32_t>("Baud", _Baud, 115200);
    Pub_IMU_Raw_Data = n.advertise<sensor_msgs::Imu>("IMU/raw_data", 1000);
    Pub_IMU_AHRS_Data = n.advertise<sensor_msgs::Imu>("IMU/AHRS_data", 1000);
    Pub_GPS_Data = n.advertise<sensor_msgs::NavSatFix>("gnss", 1000);
    Pub_Pos_Data = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    // 设置要打开的串口名称
    sp.setPort(_IMUserial);
    // 设置串口通信的波特率
    sp.setBaudrate(_Baud);
    // 串口设置timeout
    sp.setTimeout(to);
}

imu::~imu()
{
}
