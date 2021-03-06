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

    void CheckKeyFrame(Eigen::Matrix4f transform, int good_point_num, int frames_between);
    bool IsKeyFrame();
    void SetKeyFrame();

    cv::Mat GetDImage();
    cv::Mat GetRGBImage();
    std::vector<cv::KeyPoint> GetKeyPoints();
    cv::Mat GetDescriptor();
    void SetAbsPose(Eigen::Matrix4f pose);
    cv::Point3f Get3DPoint(int index);
    int GetID();
    void AddObserveIdx(int img_point_idx, int map_point_id);
    void GetObserveIdx(std::vector<std::pair<int, int>> &idx_pair_vec);

    static float _InnerCx;
    static float _InnerCy;
    static float _InnerFx;
    static float _InnerFy;
    static float _InvInnerFx;
    static float  _InvInnerFy;
    static double _DepthScale;
    static int _MaxGoodPointThres;
    static int _MaxFramesBetween;
    static int _MinFramesBetween;
    static int _key_frame_point_num;

private:
    cv::Mat _rgb_img;
    cv::Mat _d_img;
    cv::Mat _descriptor;
    bool _is_key_frame;
    int _key_frame_id;
    Eigen::Matrix4f _rel_abs_pos;
    std::vector<std::pair<int, int>> _idx_pair_vec;

    std::vector<cv::KeyPoint> _key_point_vec;
    std::mutex _is_key_frame_mtx;
    std::mutex _d_image_mtx;
    std::mutex _rgb_image_mtx;
    std::mutex _key_points_mtx;
    std::mutex _descriptor_mtx;
    std::mutex _id_mtx;
};

#endif