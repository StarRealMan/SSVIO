#ifndef __MAP_H__
#define __MAP_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Xtion_Driver.h"

class Map
{
public:

    typedef std::shared_ptr<Map> Ptr;
    
    Map(Xtion_Camera::Ptr camera);
    ~Map();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMapPointCloud();
    void UpdateMap(Eigen::Matrix4f pose);

private:

    Xtion_Camera::Ptr _mapcam;
    std::mutex _mappointcloud_mtx;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mapcloud;

    int _height;
    int _width;
    float _inner_cx;
    float _inner_cy;
    float _inner_fx;
    float _inner_fy;
    float _inv_inner_fx;
    float _inv_inner_fy;
};

#endif