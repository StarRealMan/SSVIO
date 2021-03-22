#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Xtion_Driver.h"
#include "Odometry.h"
#include "Map.h"
#include "Frame.h"

class Viewer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;
    Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry, Map::Ptr map);
    ~Viewer();


    void DrawTrajnTrans();
    void ViewerLoop();

private:
    pcl::visualization::CloudViewer _pcl_viewer;
    XtionCamera::Ptr _viewer_cam;
    Odometry::Ptr _viewer_odometry;
    Map::Ptr _viewer_map;
    Frame::Ptr _cur_frame;
};

#endif