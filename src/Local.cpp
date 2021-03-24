#include "Local.h"

Local::Local(Map::Ptr map, Config::Ptr config)
{
    _map = map;
    _WindowSize = config->GetParam<int>("WindowSize");
    
    _local_running.store(true);
    _local_thread = std::thread(std::bind(&Local::LocalLoop,this));
}

Local::~Local()
{

}

void Local::LocalLoop()
{
    while(_local_running.load())
    {
        auto t1 = std::chrono::steady_clock::now();

        // prepare data (last ${_WindowSize} keyframes)
        LocalOptimizer::Ptr optimizer(new LocalOptimizer);

        if(_map->GetKeyFrameNum() < _WindowSize)
        {
            // std::cout << "Wait for Odom Thread Ready" << std::endl;
        }
        else
        {
            int pose_num = 0;
            int measure_id = 0;
            for(size_t i = _WindowSize; i > 0; i--)
            {
                Frame::Ptr key_frame;
                key_frame = _map->GetKeyFrames(-i);

                optimizer->AddPose(key_frame->GetAbsPose(), pose_num);

                std::vector<std::pair<int, int>>& ob_vec = key_frame->GetObserve();
                for(int j = 0; j < ob_vec.size(); j++)
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
        }


        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // std::cout << "Local Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void Local::LocalStop()
{
    _local_running.store(false);
    _local_thread.join();
}
