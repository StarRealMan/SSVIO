#ifndef __LOCAL_H__
#define __LOCAL_H__

#include "Config.h"
#include "Map.h"
#include "MapPoint.h"
#include "Frame.h"
#include "Optimizer.h"

#include <iostream>
#include <vector>
#include <thread>
#include <mutex>

class  Local
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Local> Ptr;
    Local(Map::Ptr map, Config::Ptr config);
    ~Local();
    bool CheckLocalRun();
    void SetOdomStatus(bool odom_busy);
    void LocalLoop();
    void LocalStop();

private:
    Map::Ptr _map;
    int _WindowSize;
    bool odom_busy;
    int _KFInterval;
    int _last_optimize_kf_id;

    std::atomic<bool> _local_running;
    std::thread _local_thread;
    std::mutex _odom_status_mtx;
};

#endif