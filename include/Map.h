#ifndef __MAP_H__
#define __MAP_H__

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/make_shared.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

#include "Config.h"
#include "Frame.h"
#include "MapPoint.h"

class Map
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    Map(Config::Ptr config);
    ~Map();

    void Set2KeyFrameVec(Frame::Ptr key_frame);
    void Set2TrajVec(Eigen::Vector3f traj);
    Frame::Ptr GetKeyFrames(int key_frame_id);
    std::vector<Frame::Ptr>& GetKeyFramesVec();
    std::vector<Eigen::Vector3f>& GetTrajVec();
    int GetKeyFrameNum();
    MapPoint::Ptr GetMapPoint(int map_point_id);
    int GetMapPointNum();
    void TrackMapPoints(std::vector<cv::DMatch> &last_match_vec, std::vector<cv::DMatch> &this_match_vec);
    void ManageMapPoints(Frame::Ptr key_frame, std::vector<cv::DMatch>& last_match_vec);
    int InMatchVec(int i, std::vector<cv::DMatch>& last_match_vec);
    void MapPointCloudFusion();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _final_cloud;
    std::vector<Frame::Ptr> _key_frame_vec;
    std::vector<Eigen::Vector3f> _traj_vec;
    std::vector<MapPoint::Ptr> _map_point_vec;

    float _VoxelSize;

    std::mutex _key_frame_vec_mtx;
    std::mutex _traj_vec_mtx;
    std::mutex _key_frame_mtx;
    std::mutex _map_point_mtx;
};




#endif