#include "MapPoint.h"

MapPoint::MapPoint(Eigen::Vector3f map_point_pos)
{
    _map_point_id = _map_point_num;
    _map_point_num++;
    _map_point_pos = map_point_pos;
}

MapPoint::~MapPoint()
{

}

void MapPoint::SetObserve(int key_frame_id, int point_id)
{
    std::lock_guard<std::mutex> lck(_ob_mtx);
    std::pair<int, int> ob_pair;
    ob_pair.first = key_frame_id;
    ob_pair.second = point_id;
    _observed_fid_pid_vec.push_back(ob_pair);
}

std::vector<std::pair<int, int>> MapPoint::GetObserve()
{
    std::lock_guard<std::mutex> lck(_ob_mtx);
    return _observed_fid_pid_vec;
}

int MapPoint::GetID()
{
    std::lock_guard<std::mutex> lck(_id_mtx);
    return _map_point_id;
}
    
Eigen::Vector3f MapPoint::Get3DPoint()
{
    std::lock_guard<std::mutex> lck(_3d_point_mtx);
    return _map_point_pos;
}