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

        // add vertex
        // add edge
        // optimize

        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Local Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void Local::LocalStop()
{
    _local_running.store(false);
    _local_thread.join();
}
