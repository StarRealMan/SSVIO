#ifndef __MAP_POINT_H__
#define __MAP_POINT_H__

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class  MapPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    MapPoint(Eigen::Vector3f map_point_pos);
    ~MapPoint();

    void SetObserve(int key_frame_id, int point_id);
    std::vector<std::pair<int, int>>& GetObserve();
    int GetID();
    Eigen::Vector3f Get3DPoint();
    static int _map_point_num;

private:
    int _map_point_id;
    Eigen::Vector3f _map_point_pos;
    std::vector<std::pair<int, int>> _observed_fid_pid_vec; 

    std::mutex _id_mtx;
    std::mutex _ob_mtx;
    std::mutex _3d_point_mtx;
};

#endif