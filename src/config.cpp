#include "config.h"


Config::Config(std::string filename)
{
    _file = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if(_file.isOpened() == false)
    {
        std::cout << "Open config file " << filename << " failed" << std::endl;
    }
}

Config::~Config()
{
    if (_file.isOpened())
    {
        _file.release();
    }
}

template <typename T>
T Config::GetParam(std::string key)
{
    return T(_file[key]);
}