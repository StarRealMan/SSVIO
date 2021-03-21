#ifndef __VIEWER_H__
#define __VIEWER_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pangolin/pangolin.h>

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;
    Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry, Map::Ptr map);
    ~Viewer();
    void DrawTrajnTrans(std::vector<Frame::Ptr> key_frames_vec);

    void ViewerLoop();
    void ViewerStop();

private:
    XtionCamera::Ptr _viewer_cam;
    Odometry::Ptr _viewer_odometry;
    Map::Ptr _viewer_map;
    Frame::Ptr _cur_frame;
    
    std::vector<Eigen::Vector3f> _pose_traj_vec;

    std::atomic<bool> _viewer_running;
    std::thread _viewer_thread;
};

#endif