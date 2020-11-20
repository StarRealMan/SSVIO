#ifndef __FRAME_H__
#define __FRAME_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Xtion_Driver.h"
#include "g2o_optim.h"

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;
    
    Frame(){}
    Frame(Xtion_Camera::Ptr camera);
    ~Frame();

    void UpdateFrame();
    bool Optimize(Frame::Ptr lastframe);
    void getGoodMatch(std::vector<cv::DMatch>& goodmatchepoints);
    Eigen::Matrix4f getPose();
    void getFeaturepoints(std::vector<cv::KeyPoint>& featurepoints);
    void getBriefdesc(cv::Mat& briefdesc);
    Eigen::Vector3f get3DPoint(cv::Point2f imgpos);

private:

    Xtion_Camera::Ptr _framecam;
    float _inner_cx = 160.5912, _inner_cy = 120.4792,
          _inner_fx = 253.0589, _inner_fy = 254.1649,
          _inv_inner_fx = 0.003951649, _inv_inner_fy = 0.003934454;

    std::mutex _match_mtx;
    std::mutex _pose_mtx;
    std::mutex _featurepoints_mtx;
    std::mutex _briefdesc_mtx;

    cv::Mat _rgbframe;
    cv::Mat _dframe;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _rgbcloud;

    Eigen::Matrix4f _pose;
    std::vector<cv::KeyPoint> _featurepoints;
    cv::Mat _briefdesc;
    std::vector<cv::DMatch> _goodmatchepoints;
    cv::Ptr<cv::FeatureDetector> _fastdetect;
    cv::Ptr<cv::DescriptorExtractor> _briefext;
    cv::Ptr<cv::DescriptorMatcher> _bfmatcher;

};

#endif