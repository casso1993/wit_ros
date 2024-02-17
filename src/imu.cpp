#include "imu.h"

string _IMUserial;
int32_t _Baud;

void imu::process()
{
    double ax, ay, az;
    double wx, wy, wz;
    double angle_roll, angle_pitch, angle_yaw;
    double longitude, latitude, altitude;
    double GPS_Yaw, GPSV;
    float q0, q1, q2, q3;
    int satellite;
    double PDOP, HDOP, VDOP;
    geometry_msgs::Quaternion msg_q;
    geometry_msgs::Quaternion msg_AHRS_q;

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
                        signed short int temp;
                        temp = buffer[i + 3];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]);
                        ax = (temp / 32768.0) * 16 * 9.8051;
                        temp = buffer[i + 5]; // 加速度八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]); // 加速度高八位
                        ay = (temp / 32768.0) * 16 * 9.8051;
                        temp = buffer[i + 7]; // 加速度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 6]); // 加速度高八位
                        az = (temp / 32768.0) * 16 * 9.8051;
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x52) // 角速度帧
                    {
                        signed short int temp;
                        temp = buffer[i + 3];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]);
                        wx = (temp / 32768.0) * 2000;
                        temp = buffer[i + 5]; // 角速度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]); // 角速度高八位
                        wy = (temp / 32768.0) * 2000;
                        temp = buffer[i + 7]; // 角速度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 6]); // 角速度高八位
                        wz = (temp / 32768.0) * 2000;
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x53) // 角度帧
                    {
                        unsigned short int temp;
                        temp = buffer[i + 3]; // 角度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]); // 角度高八位
                        angle_roll = (temp / 32768.0) * PI;
                        temp = buffer[i + 5]; // 角度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]); // 角度高八位
                        angle_pitch = (temp / 32768.0) * PI;
                        temp = buffer[i + 7]; // 角度低八位
                        temp = temp << 8;
                        temp = (temp | buffer[i + 6]); // 角度高八位
                        angle_yaw = (temp / 32768.0) * PI;
                        msg_q = tf::createQuaternionMsgFromRollPitchYaw(angle_roll, angle_pitch, angle_yaw);
                        i += 11;
                    }
                    if (buffer[i + 1] == 0x57) // GPS帧
                    {
                        unsigned long int temp1;
                        unsigned long int temp2;
                        unsigned long int temp3;
                        unsigned long int temp4;
                        temp1 = buffer[i + 3]; //
                        temp1 = temp1 << 8;
                        temp2 = buffer[i + 4]; //
                        temp2 = temp2 << 16;
                        temp3 = buffer[i + 5]; //
                        temp3 = temp3 << 24;
                        temp4 = (temp3 | temp2 | temp1 | buffer[i + 2]);
                        longitude = temp4 / 10000000 + (double)(temp4 % 10000000) / 10000000 / 0.6;
                        temp1 = buffer[i + 7]; //
                        temp1 = temp1 << 8;
                        temp2 = buffer[i + 8]; //
                        temp2 = temp2 << 16;
                        temp3 = buffer[i + 9]; //
                        temp3 = temp3 << 24;
                        temp4 = (temp3 | temp2 | temp1 | buffer[i + 6]); //
                        latitude = temp4 / 10000000 + (double)(temp4 % 10000000) / 10000000 / 0.6;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x58) // GPS帧
                    {
                        unsigned short int temp;
                        unsigned long int temp1;
                        unsigned long int temp2;
                        unsigned long int temp3;
                        unsigned long int temp4;
                        temp = buffer[i + 3]; //
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]);
                        cout << "temp: " << temp << endl;
                        cout << "-----------------------------------" << endl;
                        altitude = (double)temp / 10.0;
                        cout << "altitude: " << altitude << endl;
                        cout << "***********************************" << endl;
                        temp = buffer[i + 5]; //
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]); //
                        GPS_Yaw = temp / 100;
                        temp1 = buffer[i + 7]; //
                        temp1 = temp1 << 8;
                        temp2 = buffer[i + 8]; //
                        temp2 = temp2 << 16;
                        temp3 = buffer[i + 9]; //
                        temp3 = temp3 << 24;
                        temp4 = (temp3 | temp2 | temp1 | buffer[i + 6]); //
                        GPSV = temp4 / 1000;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x59) // 四元数
                    {
                        signed short int temp;
                        temp = buffer[i + 3];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]);
                        q0 = temp / 32768;
                        temp = buffer[i + 5];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]);
                        q1 = temp / 32768;
                        temp = buffer[i + 7];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 6]);
                        q2 = temp / 32768;
                        temp = buffer[i + 9];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 8]);
                        q3 = temp / 32768;
                        i += 11;
                    }

                    if (buffer[i + 1] == 0x5A) // GPS精度
                    {
                        signed short int temp;
                        temp = buffer[i + 3];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 2]);
                        satellite = temp;
                        temp = buffer[i + 5];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 4]);
                        PDOP = temp / 100;
                        temp = buffer[i + 7];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 6]);
                        HDOP = temp / 100;
                        temp = buffer[i + 9];
                        temp = temp << 8;
                        temp = (temp | buffer[i + 8]);
                        VDOP = temp / 100;
                        i += 11;
                    }

                    imu_raw_data.header.stamp = ros::Time::now();
                    imu_raw_data.header.frame_id = "IMU";
                    imu_raw_data.linear_acceleration.x = ax;
                    imu_raw_data.linear_acceleration.y = ay;
                    imu_raw_data.linear_acceleration.z = az;
                    imu_raw_data.angular_velocity.x = wx;
                    imu_raw_data.angular_velocity.y = wy;
                    imu_raw_data.angular_velocity.z = wz;
                    imu_raw_data.orientation = msg_q;
                    Pub_IMU_Raw_Data.publish(imu_raw_data);

                    imu_AHRS_data.header.stamp = ros::Time::now();
                    imu_AHRS_data.header.frame_id = "IMU";
                    imu_AHRS_data.linear_acceleration.x = ax;
                    imu_AHRS_data.linear_acceleration.y = ay;
                    imu_AHRS_data.linear_acceleration.z = az;
                    imu_AHRS_data.angular_velocity.x = wx;
                    imu_AHRS_data.angular_velocity.y = wy;
                    imu_AHRS_data.angular_velocity.z = wz;
                    imu_AHRS_data.orientation.w = q0;
                    imu_AHRS_data.orientation.x = q1;
                    imu_AHRS_data.orientation.y = q2;
                    imu_AHRS_data.orientation.z = q3;
                    Pub_IMU_AHRS_Data.publish(imu_raw_data);

                    ros::Time time = ros::Time::now();
                    gnss_data.header.stamp = time;
                    gnss_data.header.frame_id = "GNSS";
                    gnss_data.latitude = latitude;
                    gnss_data.longitude = longitude;
                    gnss_data.altitude = altitude;
                    Pub_GPS_Data.publish(gnss_data);

                    DOP_data.header.stamp = time;
                    DOP_data.header.frame_id = "GNSS";
                    DOP_data.point.x = PDOP;
                    DOP_data.point.y = HDOP;
                    DOP_data.point.z = VDOP;
                    Pub_DOP_Data.publish(DOP_data);
                }
            }
        }
        boost::this_thread::sleep(boost::posix_time::millisec(10));
    }

    // 关闭串口
    sp.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Integrated_Navigation");
    new imu();
    while (true)
    {
        sleep(10);
    }
    return 0;
}

imu::imu()
{
    // 创建imu数据话题发布
    Pub_IMU_Raw_Data = n.advertise<sensor_msgs::Imu>("IMU/raw_data", 1000);
    Pub_IMU_AHRS_Data = n.advertise<sensor_msgs::Imu>("IMU/AHRS_data", 1000);
    Pub_GPS_Data = n.advertise<sensor_msgs::NavSatFix>("gnss", 1000);
    Pub_DOP_Data = n.advertise<geometry_msgs::PointStamped>("DOP", 1000);
    Pub_Pos_Data = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    nh.param<std::string>("IMUserial", _IMUserial, "/dev/ttyUSB0");
    nh.param<std::int32_t>("Baud", _Baud, 115200);

    // 设置要打开的串口名称
    sp.setPort(_IMUserial);
    // 设置串口通信的波特率
    sp.setBaudrate(_Baud);
    // 串口设置timeout
    sp.setTimeout(to);

    boost::thread *th = new boost::thread(boost::bind(&imu::process, this));
    th->detach();
}

imu::~imu()
{
}
