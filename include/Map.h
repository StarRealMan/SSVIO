#ifndef __MAP_H__
#define __MAP_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Visual_Odometry.h"

class Map
{
public:

    typedef std::shared_ptr<Map> Ptr;
    
    Map();
    ~Map();

private:
    VO::Ptr _vo;
};

#endif