#ifndef __XTION_DRIVER_H__
#define __XTION_DRIVER_H__

#include <opencv2/opencv.hpp>
#include <openni2/OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <boost/make_shared.hpp>

#include "Config.h"

class XtionCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<XtionCamera> Ptr;

    XtionCamera(Config::Ptr config);
    ~XtionCamera();

    void GrabImage();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetRGBCloud();
    cv::Mat GetDImage();
    cv::Mat GetRGBImage();
    pcl::PointXYZRGB GetRGB3DPoint(int pos_x, int pos_y, const Eigen::Matrix4f& trans);
    
    bool IsGrabRdy();
    void SetGrabRdyfalse();

    void GrabLoop();
    void CameraStop();


private:
    int _UseXtionGen;
    int _Fps;
    int _ImgHeight, _ImgWidth;
    float _InnerCx, _InnerCy, _InnerFx, _InnerFy, _InvInnerFx, _InvInnerFy;
    double _ImgDepthCoe, _DepthScale;
    openni::Array<openni::DeviceInfo> _device_list;

    cv::Mat _rgb_image;
    cv::Mat _d_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _rgb_cloud;
    bool _grab_rdy = false;
    std::mutex _rgb_image_mtx;
    std::mutex _d_image_mtx;
    std::mutex _rgb_cloud_mtx;
    std::mutex _grab_rdy_mtx;

    std::atomic<bool> _grab_running;
    std::thread _grab_thread;

    openni::VideoStream _stream_depth;
    openni::VideoStream _stream_rgb;
    openni::VideoMode _mode_d;
    openni::VideoMode _mode_rgb;

    openni::Device _camera;
    std::string _vendor = "";
    
    openni::Status rc;
    openni::VideoFrameRef _oni_d_image;
    openni::VideoFrameRef _oni_rgb_image;
    
    void ShowDevice();
    void CloseCamera();

};

#endif