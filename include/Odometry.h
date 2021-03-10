#ifndef __ODOMETRY_H__
#define __ODOMETRY_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "Config.h"
#include "Xtion_Driver.h"
#include "Feature.h"
#include "Frame.h"
#include "FeatureMatching.h"
#include "Optimizer.h"

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class Odometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Odometry> Ptr;

    Odometry(XtionCamera::Ptr camera, Config::Ptr config);
    ~Odometry();

    void OdometryLoop();
    void OdometryStop();

    Frame::Ptr GetCurFrame();
    Eigen::Matrix4f OptimizeTransform();

private:
    XtionCamera::Ptr _camera;
    ORBextractor::Ptr _orb_extractor;
    Frame::Ptr _cur_frame;
    FeatureMatching::Ptr _feature_match;
    std::mutex _cur_frame_mtx;
    std::vector<Frame::Ptr> _key_frame_vec;
    std::vector<cv::DMatch> _bow_match;

    int _final_good_point_num;
    int _frames_between;
    bool _init_rdy;
    int _FeatureNum;
    float _ScaleFactor;
    int _LevelNum;
    float _IniThFAST;
    float _MinThFAST;
    cv::Mat _InnerK;

    std::atomic<bool> _odometry_running;
    std::thread _odometry_thread;

};

#endif