#include "IMU.h"

IMU::IMU(Config::Ptr config)
{
    _IMUDevNum = config->GetParam<int>("IMUDevNum");
    _BaudRate = config->GetParam<int>("BaudRate");
    _MaxBufferLen = config->GetParam<int>("MaxBufferLen");
    _DataLen = config->GetParam<int>("DataLen");
    _DataNum = (_DataLen -4)/4;

    std::ostringstream dev_name;
    std::string dev_head = "/dev/ttyUSB";
    dev_name << dev_head << std::to_string(_IMUDevNum);

    std::cout << "logging in " << dev_name.str().c_str() << std::endl;

    _io = new boost::asio::io_service;
    _port = new boost::asio::serial_port(*_io, dev_name.str().c_str());
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
               receive_buf[2] == _HEAD3 && receive_buf[3] == _HEAD4)
            {
                float data_buf[_DataNum];

                for(int i = 0; i < _DataNum; i++)
                {
                    UcharNFloat uchar_n_float;
                    for(int j = 0; j < 4; j++)
                    {
                        uchar_n_float.ch[j] = receive_buf[4+4*i+j];
                    }
                    data_buf[i] = uchar_n_float.fl;
                }

                Eigen::Quaternionf IMU_quaternion(data_buf[0], data_buf[1], data_buf[2], data_buf[3]);
                _IMU_rotate = IMU_quaternion.matrix();
                _IMU_acc << data_buf[4], data_buf[5], data_buf[6];

                break;
            }
        }
    }
    
}

void IMU::GetIMURotateData(Eigen::Matrix3f &IMU_rotate)
{
    std::lock_guard<std::mutex> lck(_IMU_rotate_data_mtx);
    IMU_rotate = _IMU_rotate;
}

void IMU::GetIMUTransitData(Eigen::Vector3f &IMU_acc)
{
    std::lock_guard<std::mutex> lck(_IMU_transit_data_mtx);
    IMU_acc = _IMU_transit;
}

void IMU::AccIntegrate()
{
    static auto last_time = std::chrono::steady_clock::now();
    static Eigen::Vector3f last_acc = Eigen::Vector3f::Zero();
    static Eigen::Vector3f last_vel = Eigen::Vector3f::Zero();

    auto this_time = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(this_time - last_time);
    float delta_time = time_used.count();

    if(delta_time < 0.002)
    {
        delta_time = 0.002;
    }

    _IMU_vel += ((_IMU_acc+last_acc)*_Gravity/2000.0)*delta_time;
    _IMU_transit += (_IMU_vel+last_vel)*delta_time/2.0;
    
    last_vel = _IMU_vel;
    last_acc = _IMU_acc;
    last_time = this_time;
}

void IMU::IMULoop()
{
    while(_IMU_running)
    {
        auto t1 = std::chrono::steady_clock::now();
     
        IMU::ReceivePack();
        IMU::AccIntegrate();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));

        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "IMU Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void IMU::IMUStop()
{
    _IMU_running.store(false);
    _IMU_thread.join();
}
