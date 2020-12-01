#include "Frame.h"

Frame::Frame(Xtion_Camera::Ptr camera)
{
    _framecam = camera;
    _fastdetect = cv::FastFeatureDetector::create(50);
    _briefext = cv::ORB::create();
}

Frame::~Frame()
{

}

void Frame::setPose(const Eigen::Matrix4f& pose)
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    _pose = pose;
}

Eigen::Matrix4f Frame::getPose()
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    return _pose;
}

void Frame::getFeaturepoints(std::vector<cv::KeyPoint>& featurepoints)
{
    std::lock_guard<std::mutex> lck(_featurepoints_mtx);
    featurepoints = _featurepoints;
}

void Frame::getBriefdesc(cv::Mat& briefdesc)
{
    std::lock_guard<std::mutex> lck(_briefdesc_mtx);
    briefdesc = _briefdesc;
}

Eigen::Vector3f Frame::get3DPoint(const cv::Point2f& imgpos)
{
    Eigen::Matrix<float, 3, 1> temppoint;
    float depth;

    depth = _dframe.at<ushort>(imgpos.x,imgpos.y);
    temppoint << (imgpos.x - _inner_cx)*depth*_inv_inner_fx, (imgpos.y - _inner_cy)*depth*_inv_inner_fy, depth;

    return temppoint;
}

void Frame::UpdateFrame()
{
    _rgbframe = _framecam->getRGBImage();
    _dframe = _framecam->getDImage();
    _rgbcloud = _framecam->getRGBCloud();
    cv::Mat grayframe;
    cv::cvtColor(_rgbframe, grayframe, cv::COLOR_BGR2GRAY);
    _fastdetect->detect(grayframe, _featurepoints);
    _briefext->compute(grayframe, _featurepoints, _briefdesc);
}

