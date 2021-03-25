#include "Local.h"

Local::Local(Map::Ptr map, Config::Ptr config)
{
    _map = map;
    _WindowSize = config->GetParam<int>("WindowSize");
    _KFInterval = config->GetParam<int>("KFInterval");
    _last_optimize_kf_id = 0;
    
    _local_running.store(true);
    _local_thread = std::thread(std::bind(&Local::LocalLoop,this));
}

Local::~Local()
{

}

bool Local::CheckLocalRun()
{
    if(_map->GetKeyFrameNum() < _WindowSize)
    {
        return false;
    }
    else if(_map->GetOdomStatus())
    {
        return false;
    }
    else
    {
        if(_map->GetKeyFrameNum() - _last_optimize_kf_id > _KFInterval)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}


void Local::LocalLoop()
{
    while(_local_running.load())
    {
        if(CheckLocalRun())
        {
            // prepare data (last ${_WindowSize} keyframes)
            LocalOptimizer::Ptr optimizer(new LocalOptimizer);

            _map->SetLocalStatus(true);
            auto t1 = std::chrono::steady_clock::now();

            int pose_num = 0;
            int measure_id = 0;
            _last_optimize_kf_id = _map->GetKeyFrameNum();
            
            for(size_t i = _WindowSize; i > 0; i--)
            {
                Frame::Ptr key_frame;
                key_frame = _map->GetKeyFrames(-i);

                optimizer->AddPose(key_frame->GetAbsPose(), pose_num);

                std::vector<std::pair<int, int>>& ob_vec = key_frame->GetObserve();
                for(size_t j = 0; j < ob_vec.size(); j++)
                {
                    std::pair<int, int> pair = ob_vec[j];
                    std::vector<std::pair<int, int>>& point_ob = _map->GetMapPoint(pair.first)->GetObserve();

                    if(point_ob.size() > 1)
                    {
                        if(point_ob[0].first == key_frame->GetID())
                        {
                            optimizer->AddPoint(_map->GetMapPoint(pair.first)->Get3DPoint(), pair.first + _WindowSize);
                        }

                        optimizer->AddMeasure(key_frame->Get3DPoint(pair.second), measure_id, pose_num, pair.first + _WindowSize);
                        measure_id++;
                    }    
                }
                pose_num++;
            }

            optimizer->DoOptimization(20);
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            // std::cout << "Local Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
            _map->SetLocalStatus(false);
        }
        else
        {

        }

    }
}

void Local::LocalStop()
{
    _local_running.store(false);
    _local_thread.join();
}

