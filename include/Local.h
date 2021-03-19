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
    typedef std::shared_ptr<Local> Ptr;
    Local(Map::Ptr map, Config::Ptr config);
    ~Local();
    void LocalLoop();
    void LocalStop();

private:
    Map::Ptr _map;
    int _WindowSize;

    std::atomic<bool> _local_running;
    std::thread _local_thread;
};

#endif