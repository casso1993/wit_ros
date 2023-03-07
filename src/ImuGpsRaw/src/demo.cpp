//Created by ljm
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include "ImuGpsRaw/IMU_Data.h"
#include "ImuGpsRaw/GPS_Data.h"
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

using namespace std;  
using namespace Eigen;

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
  
    Eigen::Quaterniond q = rollAngle *yawAngle *pitchAngle;  
    return q;  
}  

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Integrated_Navigation");
    ros::NodeHandle n;
    //创建imu数据话题发布
    ros::Publisher Pub_IMU_Data = n.advertise<ImuGpsRaw::IMU_Data>("Wit/IMU",100);
    ros::Publisher Pub_GPS_Data = n.advertise<ImuGpsRaw::GPS_Data>("Wit/GPS",100);

    //创建一个serial对象
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(9600);
    //串口设置timeout
    sp.setTimeout(to);

    double ax,ay,az;
    double wx,wy,wz;
    double angle_roll,angle_pitch,angle_yaw;
    double longitude, latitude, altitude;
    double GPS_Yaw, GPSV;
    double q0, q1, q2, q3;
    int satellite;
    double PDOP, HDOP, VDOP;
    Eigen::Quaterniond q;
    
    ImuGpsRaw::IMU_Data imu_data;
    ImuGpsRaw::GPS_Data GPS_data;

    
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(100);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);

            for(int i=0; i<n; i++)
            {
                if(buffer[i]==0X55)            //IMU地址
                {
                        if(buffer[i+1]==0x51) //加速度帧
                        {
                            signed short int temp;
                            temp=buffer[i+3];
                            temp=temp<<8;
                            temp=(temp|buffer[i+2]);
                            ax=(temp/32768.0)*16*9.8051;
                            temp=buffer[i+5];         //加速度八位
                            temp=temp<<8;
                            temp=(temp|buffer[i+4]);  //加速度高八位
                            ay=(temp/32768.0)*16*9.8051;
                            temp=buffer[i+7];         //加速度低八位
                            temp=temp<<8;
                            temp=(temp|buffer[i+6]);  //加速度高八位
                            az=(temp/32768.0)*16*9.8051;
                            // std::cout<<"ax:"<<ax<<std::endl;
                            // std::cout<<"ay:"<<ay<<std::endl;
                            // std::cout<<"az:"<<az<<std::endl;
                            i+=11;
                        }
                        if(buffer[i+1]==0x52) //角速度帧
                        {
                            signed short int temp;
                            temp=buffer[i+3];
                            temp=temp<<8;
                            temp=(temp|buffer[i+2]);
                            wx=(temp/32768.0)*2000;
                            temp=buffer[i+5];         //角速度低八位
                            temp=temp<<8;
                            temp=(temp|buffer[i+4]);  //角速度高八位
                            wy=(temp/32768.0)*2000;
                            temp=buffer[i+7];         //角速度低八位
                            temp=temp<<8;
                            temp=(temp|buffer[i+6]);  //角速度高八位
                            wz=(temp/32768.0)*2000;
                            // std::cout<<"wx:"<<wx<<std::endl;
                            // std::cout<<"wy:"<<wy<<std::endl;
                            // std::cout<<"wz:"<<wz<<std::endl;
                            i+=11;
                        }
                     if(buffer[i+1]==0x53) //角度帧
                    {
                        unsigned short int temp;
                        temp=buffer[i+3];         //角度低八位
                        temp=temp<<8;
                        temp=(temp|buffer[i+2]);  //角度高八位
                        angle_roll=(temp/32768.0)*180;
                        temp=buffer[i+5];         //角度低八位
                        temp=temp<<8;
                        temp=(temp|buffer[i+4]);  //角度高八位
                        angle_pitch=(temp/32768.0)*180;
                        temp=buffer[i+7];         //角度低八位
                        temp=temp<<8;
                        temp=(temp|buffer[i+6]);  //角度高八位
                        angle_yaw=(temp/32768.0)*180;
                        q = euler2Quaternion(angle_roll, angle_pitch, angle_yaw) ;
                        // std::cout<<"angle_roll:"<<angle_roll<<std::endl;
                        // std::cout<<"angle_pitch:"<<angle_pitch<<std::endl;
                        // std::cout<<"angle_yaw:"<<angle_yaw<<std::endl;
                        i+=11;
                    }
                    if(buffer[i+1]==0x57) //GPS帧
                    {
                        unsigned long int temp1;
                        unsigned long int temp2;
                        unsigned long int temp3;
                        unsigned long int temp4;
                        temp1=buffer[i+3];         //
                        temp1=temp1<<8;
                        temp2=buffer[i+4];         //
                        temp2=temp2<<16;
                        temp3=buffer[i+5];         //
                        temp3=temp3<<24;
                        temp4=(temp3|temp2|temp1|buffer[i+2]);  //
                        longitude = int(temp4 /10000000)*100 + (temp4 % 10000000) / 100000;
                        temp1=buffer[i+7];         //
                        temp1=temp1<<8;
                        temp2=buffer[i+8];         //
                        temp2=temp2<<16;
                        temp3=buffer[i+9];         //
                        temp3=temp3<<24;
                        temp4=(temp3|temp2|temp1|buffer[i+6]);  //
                        latitude = int(temp4 /10000000)*100 + (temp4 % 10000000) / 100000;
                        // std::cout<<"longitude :"<<longitude <<std::endl;
                        // std::cout<<"latitude :"<<latitude <<std::endl;
                        i+=11;
                    }
                    
                    if(buffer[i+1]==0x58) //GPS帧
                    {
                        unsigned short int temp;
                        unsigned long int temp1;
                        unsigned long int temp2;
                        unsigned long int temp3;
                        unsigned long int temp4;
                        temp=buffer[i+3];         //
                        temp=temp<<8;
                        temp=(temp|buffer[i+2]);  //
                        altitude = temp / 10;
                        temp=buffer[i+5];         //
                        temp=temp<<8;
                        temp=(temp|buffer[i+4]);  //
                        GPS_Yaw = temp / 100;
                        temp1=buffer[i+7];         //
                        temp1=temp1<<8;
                        temp2=buffer[i+8];         //
                        temp2=temp2<<16;
                        temp3=buffer[i+9];         //
                        temp3=temp3<<24;
                        temp4=(temp3|temp2|temp1|buffer[i+6]);  //
                        GPSV = temp4 / 1000;
                        // std::cout<<"altitude:"<<altitude<<std::endl;
                        // std::cout<<"GPS_Yaw:"<<GPS_Yaw<<std::endl;
                        // std::cout<<"GPSV:"<<GPSV<<std::endl;
                        i+=11;
                    }

                     if(buffer[i+1]==0x59)  //四元数
                    {
                        signed short int temp;
                        temp=buffer[i+3];
                        temp=temp<<8;
                        temp=(temp|buffer[i+2]);
                        q0 = temp / 32768;
                        temp=buffer[i+5];        
                        temp=temp<<8;
                        temp=(temp|buffer[i+4]);  
                        q1 = temp / 32768;
                        temp=buffer[i+7];       
                        temp=temp<<8;
                        temp=(temp|buffer[i+6]);  
                        q2 = temp / 32768;
                        temp=buffer[i+9];         
                        temp=temp<<8;
                        temp=(temp|buffer[i+8]);  
                        q3 = temp / 32768;
                        i+=11;
                    }

                    if(buffer[i+1]==0x5A) //GPS精度
                    {
                        signed short int temp;
                        temp=buffer[i+3];
                        temp=temp<<8;
                        temp=(temp|buffer[i+2]);
                        satellite = temp;
                        temp=buffer[i+5];         
                        temp=temp<<8;
                        temp=(temp|buffer[i+4]);  
                        PDOP = temp / 100;
                        temp=buffer[i+7];         
                        temp=temp<<8;
                        temp=(temp|buffer[i+6]);  
                        HDOP = temp / 100;
                        temp=buffer[i+9];         
                        temp=temp<<8;
                        temp=(temp|buffer[i+8]);  
                        VDOP = temp / 100;
                        i+=11;
                    }

                    imu_data.IMU_Data.header.stamp = ros::Time::now();
                    imu_data.IMU_Data.header.frame_id = "base_link";
                    imu_data.IMU_Data.linear_acceleration.x = ax;
                    imu_data.IMU_Data.linear_acceleration.y = ay;
                    imu_data.IMU_Data.linear_acceleration.z = az;
                    imu_data.IMU_Data.angular_velocity.x = wx; 
                    imu_data.IMU_Data.angular_velocity.y = wy; 
                    imu_data.IMU_Data.angular_velocity.z = wz;
                    imu_data.IMU_Data.orientation.x = q.x();
                    imu_data.IMU_Data.orientation.y = q.y();
                    imu_data.IMU_Data.orientation.z = q.z();
                    imu_data.IMU_Data.orientation.w = q.w();
                    imu_data.Roll = angle_roll;
                    imu_data.Pitch = angle_pitch;
                    imu_data.Yaw = angle_yaw;
                    Pub_IMU_Data.publish(imu_data);

                    GPS_data.GPS_Data.header.stamp =ros::Time::now();
                    GPS_data.GPS_Data.header.frame_id = "base_link";
                    GPS_data.GPS_Data.latitude = latitude;
                    GPS_data.GPS_Data.longitude = longitude;
                    GPS_data.GPS_Data.altitude = altitude;
                    GPS_data.DOPP = PDOP;
                    GPS_data.DOPH = HDOP;
                    GPS_data.DOPV = VDOP;
                    GPS_data.satellite = satellite;
                    GPS_data.GPS_Yaw = GPS_Yaw;
                    GPS_data.q0 = q0;
                    GPS_data.q1 = q1;
                    GPS_data.q2 = q2;
                    GPS_data.q3 = q3;
                    Pub_GPS_Data.publish(GPS_data);

                }
            }
            // std::cout << std::endl;
            //sp.write(buffer, n);
        }
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}
