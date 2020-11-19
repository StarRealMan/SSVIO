#include "Frame.h"

Frame::Frame(Xtion_Camera::Ptr camera)
{
    _framecam = camera;
    _fastdetect = cv::FastFeatureDetector::create(50);
    _briefext = cv::ORB::create();
    _bfmatcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
}

Frame::~Frame()
{

}

void Frame::getGoodMatch(std::vector<cv::DMatch>& goodmatchepoints)
{
    std::lock_guard<std::mutex> lck(_match_mtx);
    goodmatchepoints = _goodmatchepoints;
}

Eigen::Matrix4f Frame::getPose()
{
    std::lock_guard<std::mutex> lck(_pose_mtx);
    return _pose;
}

void Frame::UpdateFrame()
{
    _rgbframe = _framecam->getRGBImage();
    _dframe = _framecam->getDImage();
    _rgbcloud = _framecam->getRGBCloud();
    _goodmatchepoints.clear();
    FeatureDetect();
}

void Frame::Optimize(Frame::Ptr lastframe)
{
    Eigen::Matrix4f pose_estimate;

    std::vector<cv::DMatch> matchepoints;
    _bfmatcher->match(_briefdesc, lastframe->_briefdesc, matchepoints);
    
    if(!matchepoints.empty())
    {
        auto min_max = std::minmax_element(matchepoints.begin(), matchepoints.end(),
                                [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
        double min_dist = min_max.first->distance;
        for (int i = 0; i < matchepoints.size(); i++)
        {
            if (matchepoints[i].distance <= std::max(2 * min_dist, 30.0))
            {
                _goodmatchepoints.push_back(matchepoints[i]);
            }
        }
    }
    else
    {
        std::cout << "match failed" << std::endl;
    }
    
    
    // pose_estimate
    _pose = pose_estimate;
}

void Frame::FeatureDetect()
{
    cv::Mat grayframe;
    cv::cvtColor(_rgbframe, grayframe, cv::COLOR_BGR2GRAY);
    _fastdetect->detect(grayframe, _featurepoints);
    _briefext->compute(grayframe, _featurepoints, _briefdesc);
}

