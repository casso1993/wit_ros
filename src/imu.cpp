#include "imu.h"

int imu::open_serial(string port, int baud_rate)
{
    serial_fd = open(port.c_str(), O_RDWR | O_NOCTTY); // 打开串口
    if (serial_fd == -1)
    {
        cout << "串口打开失败！" << endl;
        return -1;
    }
    else
    {
        cout << "串口打开成功！" << endl;
    }
    struct termios options; // 串口配置结构体
    tcgetattr(serial_fd, &options);
    options.c_cflag |= CLOCAL;      // 忽略调制解调器状态行
    options.c_cflag |= CREAD;       // 启用接收器
    options.c_cflag &= ~CSIZE;      // 字符长度掩码。取值为CS5, CS6, CS7或CS8
    options.c_cflag |= CS8;         // 8位数据位
    options.c_cflag &= ~PARENB;     // 校验位
    options.c_cflag &= ~CSTOPB;     // 停止位
    options.c_iflag |= IGNPAR;      // 忽略帧错误和奇偶校验错
    options.c_oflag = 0;            // 输出模式
    options.c_lflag = 0;            // 不激活终端模式
    options.c_cc[VTIME] = 0;        // 读取一个字符等待1*(1/10)s
    options.c_cc[VMIN] = 1;         // 读取字符的最少个数为1
    cfsetospeed(&options, B115200); // 设置波特率为115200
    cfsetispeed(&options, B115200);
    tcflush(serial_fd, TCIFLUSH);                     // 清空输入缓存区
    if (tcsetattr(serial_fd, TCSANOW, &options) != 0) // TCSANOW：不等数据传输完毕就立即改变属性
    {
        cout << "串口设置失败！" << endl;
        return -1;
    }
    else
    {
        cout << "串口设置成功！" << endl;
    }
    return 0;
}

void imu::process()
{
    open_serial(_IMUserial, _Baud);
    while (ros::ok())
    {
        read(serial_fd, buffer, 1);
        if (buffer[0] == 0x55)
        {
            read(serial_fd, buffer + 1, 1);
            switch (buffer[1])
            {
            case 0x51:
            {
                read(serial_fd, ax, 9);
                imu_raw_msg.linear_acceleration.x = ax[0] / 32768.0 * 16.0 * 9.80665; // 填充imu_msg的线性加速度
                imu_raw_msg.linear_acceleration.y = ax[1] / 32768.0 * 16.0 * 9.80665;
                imu_raw_msg.linear_acceleration.z = ax[2] / 32768.0 * 16.0 * 9.80665;
                imu_AHRS_msg.linear_acceleration.x = ax[0] / 32768.0 * 16.0 * 9.80665; // 填充imu_msg的线性加速度
                imu_AHRS_msg.linear_acceleration.y = ax[1] / 32768.0 * 16.0 * 9.80665;
                imu_AHRS_msg.linear_acceleration.z = ax[2] / 32768.0 * 16.0 * 9.80665;
                Pub_IMU_Raw_Data.publish(imu_raw_msg); // 发布imu消息
                Pub_IMU_AHRS_Data.publish(imu_AHRS_msg);
                break;
            }
            case 0x52:
            {
                read(serial_fd, gx, 9);
                imu_raw_msg.angular_velocity.x = gx[0] / 32768.0 * 2000 * 3.1415926 / 180.0; // 填充imu_msg的角速度
                imu_raw_msg.angular_velocity.y = gx[1] / 32768.0 * 2000 * 3.1415926 / 180.0;
                imu_raw_msg.angular_velocity.z = gx[2] / 32768.0 * 2000 * 3.1415926 / 180.0;
                imu_AHRS_msg.angular_velocity.x = gx[0] / 32768.0 * 2000 * 3.1415926 / 180.0; // 填充imu_msg的角速度
                imu_AHRS_msg.angular_velocity.y = gx[1] / 32768.0 * 2000 * 3.1415926 / 180.0;
                imu_AHRS_msg.angular_velocity.z = gx[2] / 32768.0 * 2000 * 3.1415926 / 180.0;
                Pub_IMU_Raw_Data.publish(imu_raw_msg);   // 发布imu消息
                Pub_IMU_AHRS_Data.publish(imu_AHRS_msg); // 发布imu消息
                break;
            }
            case 0x53:
            {
                read(serial_fd, sAngle, 9);
                imu_raw_msg.header.stamp = ros::Time::now(); // 填充imu_msg的头部时间戳
                imu_raw_msg.header.frame_id = "IMU";
                double roll = sAngle[0] / 32768.0 * PI;
                double pitch = sAngle[1] / 32768.0 * PI;
                double yaw = sAngle[2] / 32768.0 * PI;

                geometry_msgs::Quaternion msg_q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
                switch (_IMU_type)
                {
                case 1:
                    imu_raw_msg.orientation = msg_q;
                    Pub_IMU_Raw_Data.publish(imu_raw_msg); // 发布imu消息
                    if (_IsPose_On)
                    {
                        pose_data.header.stamp = ros::Time::now(); // 填充imu_msg的头部时间戳
                        pose_data.header.frame_id = "IMU";
                        pose_data.pose.orientation = msg_q;
                        Pub_Pos_Data.publish(pose_data);
                    }
                    break;

                case 2:
                    imu_raw_msg.orientation.x = -msg_q.x;
                    imu_raw_msg.orientation.y = msg_q.y;
                    imu_raw_msg.orientation.z = msg_q.z;
                    imu_raw_msg.orientation.w = msg_q.w;
                    Pub_IMU_Raw_Data.publish(imu_raw_msg); // 发布imu消息
                    if (_IsPose_On)
                    {
                        pose_data.header.stamp = ros::Time::now(); // 填充imu_msg的头部时间戳
                        pose_data.header.frame_id = "IMU";
                        pose_data.pose.orientation.x = -msg_q.x;
                        pose_data.pose.orientation.y = msg_q.y;
                        pose_data.pose.orientation.z = msg_q.z;
                        pose_data.pose.orientation.w = msg_q.w;
                        Pub_Pos_Data.publish(pose_data);
                    }
                    break;
                }
                break;
            }
            case 0x57:
            {
                read(serial_fd, GNSS1, 9);
                unsigned int longitude = (unsigned int)(GNSS1[1] << 16) | (unsigned int)(GNSS1[0]);
                unsigned int latitude = (unsigned int)(GNSS1[3] << 16) | (unsigned int)(GNSS1[2]);
                gnss_msg.longitude = longitude / 10000000 + (double)(longitude % 10000000) / 10000000 / 0.6;
                gnss_msg.latitude = latitude / 10000000 + (double)(latitude % 10000000) / 10000000 / 0.6;
                Pub_GNSS_Data.publish(gnss_msg);
                break;
            }
            case 0x58:
            {
                read(serial_fd, GNSS2, 9);
                unsigned int altitude = (unsigned int)(GNSS2[1] << 16) | (unsigned int)(GNSS2[0]);
                unsigned int GPS_Yaw = (unsigned int)(GNSS2[3] << 16) | (unsigned int)(GNSS2[2]);
                gnss_msg.altitude = altitude / 10000000 + (double)(altitude % 10000000) / 10000000 / 0.6;
                Pub_GNSS_Data.publish(gnss_msg);
                break;
            }
            case 0x59:
            {
                read(serial_fd, Quat, 9);
                imu_AHRS_msg.header.stamp = ros::Time::now();
                imu_AHRS_msg.header.frame_id = "IMU";
                imu_AHRS_msg.orientation.w = Quat[0] / 32768.0;
                imu_AHRS_msg.orientation.x = Quat[1] / 32768.0;
                imu_AHRS_msg.orientation.y = Quat[2] / 32768.0;
                imu_AHRS_msg.orientation.z = Quat[3] / 32768.0;
                Pub_IMU_AHRS_Data.publish(imu_AHRS_msg);
                break;
            }
            case 0x5A:
            {
                read(serial_fd, DOP, 9);
                gnss_msg.header.stamp = ros::Time::now();
                gnss_msg.header.frame_id = "GNSS";
                gnss_msg.position_covariance[0] = DOP[0];
                gnss_msg.position_covariance[4] = DOP[1];
                gnss_msg.position_covariance[8] = DOP[2];
                Pub_GNSS_Data.publish(gnss_msg);
                break;
            }
            }
        }
        // ros::spinOnce();          // 处理ROS的信息，比如订阅消息,并调用回调函数
        // loop_rate.sleep();        // 按照循环频率延时
    }
    close(serial_fd);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Integrated_Navigation");
    imu imu;
    imu.process();
    return 0;
}

imu::imu()
{
    ros::NodeHandle nh("~");
    nh.param<std::string>("IMUserial", _IMUserial, "/dev/ttyUSB0");
    nh.param<std::int32_t>("Baud", _Baud, 115200);
    nh.param<bool>("IsPose_On", _IsPose_On, 0);
    nh.param<int>("IMU_type", _IMU_type, 1);
    Pub_IMU_Raw_Data = n.advertise<sensor_msgs::Imu>("IMU/raw_data", 1000);
    Pub_IMU_AHRS_Data = n.advertise<sensor_msgs::Imu>("IMU/AHRS_data", 1000);
    Pub_GNSS_Data = n.advertise<sensor_msgs::NavSatFix>("GNSS", 1000);
    if (_IsPose_On)
    {
        Pub_Pos_Data = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
    }
}

imu::~imu()
{
}