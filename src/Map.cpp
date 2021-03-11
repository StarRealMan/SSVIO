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

Frame::Ptr Map::GetKeyFrames(int key_frame_num)
{
    std::lock_guard<std::mutex> lck(_key_frame_mtx);
    int key_frame_vec_size = _key_frame_vec.size();

    if(key_frame_num < 0)
    {
        return _key_frame_vec[key_frame_vec_size + key_frame_num];
    }
    else
    {
        return _key_frame_vec[key_frame_num];
    }
}

int Map::GetKeyFrameNum()
{
    return _key_frame_vec.size();
}

int Map::GetMapPointNum()
{
    return _map_point_vec.size();
}

void Map::TrackMapPoints(std::vector<cv::DMatch> &match_vec)
{
    Frame::Ptr cur_key_frame = GetKeyFrames(-1);
    Frame::Ptr last_key_frame = GetKeyFrames(-2);

    if(_map_point_vec.size() == 0)
    {
        for(int i = 0; i < match_vec.size(); i++)
        {
            MapPoint::Ptr map_point(new MapPoint(cur_key_frame->Get3DPoint(match_vec[i].queryIdx)));
            map_point->SetObserve(last_key_frame->GetID());
            map_point->SetObserve(cur_key_frame->GetID());
            _map_point_vec.push_back(map_point);
            cur_key_frame->AddObserveIdx(match_vec[i].queryIdx, map_point->GetID());
        }
    }
    else
    {
        std::vector<std::pair<int, int>> idx_pair_vec;
        last_key_frame->GetObserveIdx(idx_pair_vec);
        std::cout << "Last frame observed point num: " << idx_pair_vec.size() << std::endl;
        std::cout << "Cur frame match point num: " << match_vec.size() << std::endl;
            
        for(int i = 0; i < match_vec.size(); i++)
        {
            bool is_point_observed = false;
            for(int j = 0; j < idx_pair_vec.size(); j++)
            {
                if(match_vec[i].trainIdx == idx_pair_vec[j].first)
                {
                    _map_point_vec[idx_pair_vec[j].second]->SetObserve(cur_key_frame->GetID());
                    cur_key_frame->AddObserveIdx(match_vec[i].queryIdx, idx_pair_vec[j].second);
                    is_point_observed = true;
                    break;
                }
            }
            if(!is_point_observed)
            {
                MapPoint::Ptr map_point(new MapPoint(cur_key_frame->Get3DPoint(match_vec[i].queryIdx)));
                map_point->SetObserve(cur_key_frame->GetID());
                _map_point_vec.push_back(map_point);
                cur_key_frame->AddObserveIdx(match_vec[i].queryIdx, map_point->GetID());
            }
        }
     }
    
}

