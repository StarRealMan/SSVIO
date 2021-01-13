#ifndef __MAP_H__
#define __MAP_H__

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

#include "Xtion_Driver.h"
#include "config.h"

class Map
{
public:

    typedef std::shared_ptr<Map> Ptr;
    
    Map(Xtion_Camera::Ptr camera, Config::Ptr config);
    ~Map();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getMapPointCloud();
    void setPose(const Eigen::Matrix4f& pose);
    void PointcloudTransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud,const Eigen::Matrix4f& trans);
    void UpdateMap(void);
    void MapLoop(void);
    void MapStop();

private:

    Xtion_Camera::Ptr _mapcam;
    std::mutex _pose_mtx;
    std::mutex _mappointcloud_mtx;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _mapcloud;
    Eigen::Matrix4f _current_pose;

    bool _new_pos_set;
    std::atomic<bool> _maprunning;
    std::thread _mapthread;

    int _height;
    int _width;
    float _inner_cx;
    float _inner_cy;
    float _inner_fx;
    float _inner_fy;
    float _inv_inner_fx;
    float _inv_inner_fy;
    float _voxel_size;
};

#endif