#ifndef __XTION_DRIVER_H__
#define __XTION_DRIVER_H__

#include <opencv2/opencv.hpp>
#include <openni2/OpenNI.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class Xtion_Camera
{
public:
    typedef std::shared_ptr<Xtion_Camera> Ptr;

    Xtion_Camera();
    ~Xtion_Camera();

    void grab();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRGBCloud();
    cv::Mat getDImage();
    cv::Mat getRGBImage();
    pcl::PointXYZRGB getRGB3DPoint(int pos_x, int pos_y, Eigen::Matrix4f trans);
    
    bool isGrabRdy();

    void GrabLoop();
    void GrabStop();


private:
    int _height,_width;
    float _inner_cx,_inner_cy,_inner_fx,_inner_fy,_inv_inner_fx,_inv_inner_fy;
    openni::Array<openni::DeviceInfo> _deviceList;

    cv::Mat _rgbImage;
    cv::Mat _dImage;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _rgbCloud;
    bool _grabRdy = false;
    std::mutex _rgbimage_mtx;
    std::mutex _dimage_mtx;
    std::mutex _rgbcloud_mtx;
    std::mutex _grabrdy_mtx;

    std::atomic<bool> _grabrunning;
    std::thread _grabthread;

    openni::VideoStream _streamDepth;
    openni::VideoStream _streamRGB;
    openni::VideoMode _modeD;
    openni::VideoMode _modeRGB;

    openni::Device _camera;
    std::string vendor = "";
    
    openni::Status rc;
    openni::VideoFrameRef _onidImage;
    openni::VideoFrameRef _onirgbImage;
    
    void showdevice();
    void closecamera();

};

#endif