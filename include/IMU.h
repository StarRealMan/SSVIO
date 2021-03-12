#ifndef __IMU_H__
#define __IMU_H__

#include <boost/asio.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Config.h"

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class IMU
{
public:
    typedef std::shared_ptr<IMU> Ptr;

    IMU(Config::Ptr config);
    ~IMU();
    void send(unsigned char* ch, size_t length);
    void receive(unsigned char* buff, size_t& length);
    void ReceivePack();
    void GetIMUData(Eigen::Matrix3f &IMU_rotate, Eigen::Vector3f &IMU_acc);

    void IMULoop();
    void IMUStop();

private:
    int _IMUDevNum;
    int _BaudRate;
    boost::asio::io_service* _io;
    boost::asio::serial_port* _port;
    int _MaxBufferLen;
    int _DataLen;
    int _DataNum;

    Eigen::Quaternionf _IMU_quaternion;
    Eigen::Matrix3f _IMU_rotate;
    Eigen::Vector3f _IMU_acc;

    std::atomic<bool> _IMU_running;
    std::thread _IMU_thread;
    std::mutex _IMU_data_mtx;

    union UcharNFloat
    {
        float fl;
        unsigned char ch[4];
    };

    static const unsigned char _HEAD1 = 0x55;
    static const unsigned char _HEAD2 = 0x00;
    static const unsigned char _TAIL1 = 0x00;
    static const unsigned char _TAIL2 = 0xAA;
};

#endif


