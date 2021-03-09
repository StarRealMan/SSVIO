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
#include "Odometry.h"
#include "Frame.h"

class Viewer
{
public:
    typedef std::shared_ptr<Viewer> Ptr;
    Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry);
    ~Viewer();

    char GetKeyVal();
    void ViewerLoop();
    void ViewerStop();

private:
    char _key_val;
    pcl::visualization::CloudViewer _pcl_viewer;
    XtionCamera::Ptr _viewer_cam;
    Odometry::Ptr _viewer_odometry;

    std::mutex _key_val_mtx;
    std::mutex _visualizer_mtx;
    std::atomic<bool> _viewer_running;
    std::thread _viewer_thread;
    Frame::Ptr _cur_frame;

};

#endif