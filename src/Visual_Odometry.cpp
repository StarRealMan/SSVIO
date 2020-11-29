#include "Visual_Odometry.h"

VO::VO(Xtion_Camera::Ptr camera):_frame(new Frame(camera)),_lastframe(new Frame(_vocam))//_map(new Map)
{
    _vocam = camera;

    _poses.push_back(Eigen::Matrix4f::Identity());
    _vorunning.store(true);
    _vothread = std::thread(std::bind(&VO::VOLoop,this));
}

VO::~VO()
{

}

void VO::VOLoop()
{
    std::vector<cv::DMatch> goodmatch;
    while(_vorunning.load())
    {
        if(_vocam->isGrabRdy())
        {
            auto t1 = std::chrono::steady_clock::now();
            // std::cout << "vo start" << std::endl;
            _vocam->setGrabRdyfalse();
            Eigen::Matrix4f pose_estimate;
            _frame->UpdateFrame();
            
            std::cout << "<=============================================================>" << std::endl;
            
            if(!_InitRdy)
            {
                std::cout << "Initializing......" << std::endl;
                _InitRdy = true;
            }
            else
            {
                _frame->Optimize(_lastframe);
                //_frame->getGoodMatch(goodmatch);
                _poses.push_back(_poses[-1]*_frame->getPose());
                // Map
            }
            memcpy(_lastframe.get(),_frame.get(),sizeof(Frame));
            // std::cout << "vo ok" << std::endl;
            auto t2 = std::chrono::steady_clock::now();
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
            std::cout << time_used.count()*1000 << " ms per frame " << std::endl;
        }
    }
}

void VO::VOStop()
{
    _vorunning.store(false);
    _vothread.join();
}