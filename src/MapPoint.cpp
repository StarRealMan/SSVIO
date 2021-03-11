#include "MapPoint.h"

MapPoint::MapPoint(cv::Point3f map_point_pos)
{
    _map_point_id = _map_point_num;
    _map_point_num++;
    _map_point_pos = map_point_pos;
}

MapPoint::~MapPoint()
{

}

void MapPoint::SetObserve(int key_frame_id)
{
    _observed_key_frame_id.push_back(key_frame_id);
}

int MapPoint::GetID()
{
    std::lock_guard<std::mutex> lck(_id_mtx);
    return _map_point_id;
}
    
