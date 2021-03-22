#include "Map.h"

Map::Map()
{

}

Map::~Map()
{

}

void Map::Set2KeyFrameVec(Frame::Ptr key_frame)
{
    _key_frame_vec.push_back(key_frame);
}

Frame::Ptr Map::GetKeyFrames(int key_frame_id)
{
    std::lock_guard<std::mutex> lck(_key_frame_mtx);
    int key_frame_vec_size = _key_frame_vec.size();

    if(key_frame_id < 0)
    {
        return _key_frame_vec[key_frame_vec_size + key_frame_id];
    }
    else
    {
        return _key_frame_vec[key_frame_id];
    }
}

std::vector<Frame::Ptr> Map::GetKeyFramesVec()
{
    std::lock_guard<std::mutex> lck(_key_frame_mtx);
    return _key_frame_vec;
}

int Map::GetKeyFrameNum()
{
    return _key_frame_vec.size();
}

MapPoint::Ptr Map::GetMapPoint(int map_point_id)
{
    std::lock_guard<std::mutex> lck(_map_point_mtx);
    return _map_point_vec[map_point_id];
}

int Map::GetMapPointNum()
{
    return _map_point_vec.size();
}

void Map::TrackMapPoints(std::vector<cv::DMatch> &last_match_vec, std::vector<cv::DMatch> &this_match_vec)
{
    std::vector<cv::DMatch> temp_last_match_vec;

    for(int i = 0; i < last_match_vec.size(); i++)
    {
        for(int j = 0; j < this_match_vec.size(); j++)
        {
            if(last_match_vec[i].queryIdx == this_match_vec[j].trainIdx)
            {
                cv::DMatch temp_match;

                temp_match.trainIdx = last_match_vec[i].trainIdx;
                temp_match.queryIdx = this_match_vec[j].queryIdx;

                temp_last_match_vec.push_back(temp_match);
                break;
            }
        }
    }

    last_match_vec.clear();
    last_match_vec = temp_last_match_vec;
}

void Map::ManageMapPoints(Frame::Ptr key_frame, std::vector<cv::DMatch> last_match_vec)
{
    Eigen::Matrix4f world_trans = key_frame->GetAbsPose().inverse();
    for(int i = 0; i < key_frame->GetKeyPoints().size(); i++)
    {
        int match_num = InMatchVec(i, last_match_vec);
        if(match_num == -1)
        {
            cv::Point3f cv_point_pose = key_frame->Get3DPoint(i);
            Eigen::Vector3f eigen_point_pose(cv_point_pose.x, cv_point_pose.y, cv_point_pose.z);
            MapPoint::Ptr map_point = std::make_shared<MapPoint>(world_trans.block<3,3>(0,0)*eigen_point_pose+
                                                                 world_trans.block<3,1>(0,3));
            map_point->SetObserve(key_frame->GetID(), i);
            _map_point_vec.push_back(map_point);
            key_frame->SetObserve(map_point->GetID(), i);
        }
        else
        {
            MapPoint::Ptr map_point = _map_point_vec[GetKeyFrames(-1)->GetMapPointID(last_match_vec[match_num].queryIdx)];

            map_point->SetObserve(key_frame->GetID(), i);
            key_frame->SetObserve(map_point->GetID(), i);
        }
    }
}

int Map::InMatchVec(int i, std::vector<cv::DMatch> last_match_vec)
{
    for(int j = 0; j < last_match_vec.size(); j++)
    {
        if(last_match_vec[j].queryIdx == i)
        {
            return j;
        }
    }

    return -1;
}
