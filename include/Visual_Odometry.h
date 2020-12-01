#ifndef __VISUAL_ODOMETRY_H__
#define __VISUAL_ODOMETRY_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Xtion_Driver.h"
#include "Frame.h"
#include "Map.h"
#include "config.h"

class VO
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VO> Ptr;

    VO(Xtion_Camera::Ptr camera, Config::Ptr config);
    VO(){};
    ~VO();

    pcl::PointXYZRGB NewPoint_Generate(ushort u, ushort v);
    Eigen::Matrix4f Optimize();
    void VOLoop();
    void VOStop();

private:
    Xtion_Camera::Ptr _vocam;
    Frame::Ptr _frame;
    Frame::Ptr _lastframe;
    Map::Ptr _map;

    bool _InitRdy = false;

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> _poses;

    std::atomic<bool> _vorunning;
    std::thread _vothread;
    cv::Ptr<cv::DescriptorMatcher> _bfmatcher;
    std::vector<cv::DMatch> _goodmatchepoints;

    float _featurepoint_coe;
    float _featurepoint_max;
    int _goodmatch_thresh;
    int _optim_round;
};

#endif