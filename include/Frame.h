#ifndef __FRAME_H__
#define __FRAME_H__

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Feature.h"

class Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    Frame(cv::Mat rgb_img, cv::Mat d_img, ORBextractor::Ptr _orb_extractor);
    ~Frame();

    void CheckKeyFrame(Eigen::Matrix4f transform);
    bool IsKeyFrame();
    void SetKeyFrame();

    cv::Mat GetDImage();
    cv::Mat GetRGBImage();
    std::vector<cv::KeyPoint> GetKeyPoints();
    cv::Mat GetDescriptor();
    void SetAbsPose(Eigen::Matrix4f pose);
    cv::Point3f Get3DPoint(int index);

    static float _InnerCx, _InnerCy, _InnerFx, _InnerFy, _InvInnerFx, _InvInnerFy;
    static double _DepthScale;

private:
    cv::Mat _rgb_img;
    cv::Mat _d_img;
    cv::Mat _descriptor;
    bool _is_key_frame;
    Eigen::Matrix4f _rel_abs_pos;

    std::vector<cv::KeyPoint> _key_point_vec;
    std::mutex _is_key_frame_mtx;
    std::mutex _d_image_mtx;
    std::mutex _rgb_image_mtx;
    std::mutex _key_points_mtx;
    std::mutex _descriptor_mtx;

};

#endif