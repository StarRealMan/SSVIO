#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class Config{

public:

    typedef std::shared_ptr<Config> Ptr;

    Config(const std::string& filename);
    ~Config();

    template <typename T>
    T GetParam(const std::string& key)
    {
        return T(_file[key]);
    }

private:
    
    cv::FileStorage _file;

};



#endif