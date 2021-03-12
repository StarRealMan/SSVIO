#include "IMU.h"

IMU::IMU(Config::Ptr config)
{
    _IMUDevNum = config->GetParam<int>("IMUDevNum");
    _BaudRate = config->GetParam<int>("BaudRate");
    _MaxBufferLen = config->GetParam<int>("MaxBufferLen");
    _DataLen = config->GetParam<int>("DataLen");
    _DataNum = (_DataLen - 4)/4;

    _io = new boost::asio::io_service;
    _port = new boost::asio::serial_port(*_io, "COM1");
    _port->set_option(boost::asio::serial_port::baud_rate(_BaudRate));
    _port->set_option(boost::asio::serial_port::flow_control());
    _port->set_option(boost::asio::serial_port::parity());
    _port->set_option(boost::asio::serial_port::stop_bits());
    _port->set_option(boost::asio::serial_port::character_size(8));

    _IMU_running.store(true);
    _IMU_thread = std::thread(std::bind(&IMU::IMULoop,this));
}

IMU::~IMU()
{
    delete _io;
    delete _port;
}

void IMU::send(unsigned char* ch, size_t length)
{
    boost::asio::write(*_port, boost::asio::buffer(ch, length));
}


void IMU::receive(unsigned char* buff, size_t& length)
{
    boost::system::error_code err;
    length = _port->read_some(boost::asio::buffer(buff, _MaxBufferLen), err);
}

void IMU::ReceivePack()
{
    while(1)
    {
        unsigned char receive_buf[_MaxBufferLen];
        size_t length;
        receive(receive_buf, length);
        if(length == _DataLen)
        {
            if(receive_buf[0] == _HEAD1 && receive_buf[1] == _HEAD2 &&
               receive_buf[_DataLen-2] == _TAIL1 && receive_buf[_DataLen-1] == _TAIL2)
            {
                unsigned char data_buf[_DataNum];
                for(int i = 0; i < _DataNum; i++)
                {
                    UcharNFloat uchar_n_float;
                    for(int j = 0; j < 4; j++)
                    {
                        uchar_n_float.ch[j] = receive_buf[2+4*i+j];
                    }
                    data_buf[i] = uchar_n_float.fl;
                }

                Eigen::Quaternionf IMU_quaternion(data_buf[0], data_buf[1], data_buf[2], data_buf[3]);
                _IMU_rotate = IMU_quaternion.matrix() * _IMU_rotate.inverse();
                _IMU_acc << data_buf[4], data_buf[5], data_buf[6];

                break;
            }
        }
    }
    
}

void IMU::GetIMUData(Eigen::Matrix3f &IMU_rotate, Eigen::Vector3f &IMU_acc)
{
    std::lock_guard<std::mutex> lck(_IMU_data_mtx);
    IMU_rotate = _IMU_rotate;
    IMU_acc = _IMU_acc;
}

void IMU::IMULoop()
{
    while(_IMU_running)
    {
        IMU::ReceivePack();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void IMU::IMUStop()
{
    _IMU_running.store(false);
    _IMU_thread.join();
}
