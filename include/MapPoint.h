#ifndef __MAP_POINT_H__
#define __MAP_POINT_H__

#include <opencv2/opencv.hpp>

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class  MapPoint
{
public:
    typedef std::shared_ptr<MapPoint> Ptr;
    MapPoint(cv::Point3f map_point_pos);
    ~MapPoint();

    void SetObserve(int key_frame_id);
    int GetID();

    static int _map_point_num;
    
private:
    int _map_point_id;
    cv::Point3f _map_point_pos;
    std::vector<int> _observed_key_frame_id;

    std::mutex _id_mtx;
};

#endif