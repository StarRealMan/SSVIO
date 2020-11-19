#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Xtion_Driver.h"

class MyViewer
{
public:
    typedef std::shared_ptr<MyViewer> Ptr;
    MyViewer(Xtion_Camera::Ptr camera);
    ~MyViewer();

    char getKeyVal();
    void ViewerLoop();
    void ViewerStop();

private:
    char _keyVal;
    pcl::visualization::CloudViewer _pclviewer;
    Xtion_Camera::Ptr _viewerCam;

    std::mutex _keyval_mtx;
    std::mutex _visualizer_mtx;
    std::atomic<bool> _viewerrunning;
    std::thread _viewerthread;

};

#endif