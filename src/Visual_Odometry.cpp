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
        if(_vocam->isGrabRdy())// 线程同步问题
        {
            std::cout << " vo start " << std::endl;

            Eigen::Matrix4f pose_estimate;
            _frame->UpdateFrame();
            if(!_InitRdy)
            {
                _InitRdy = true;
            }
            else
            {
                _frame->Optimize(_lastframe);
                _frame->getGoodMatch(goodmatch);
                std::cout << goodmatch.size() << std::endl;
                _poses.push_back(_poses[-1]*_frame->getPose());
                // Map
            }
            memcpy(_lastframe.get(),_frame.get(),sizeof(Frame));
            std::cout << " vo ok " << std::endl;
        }
    }
}

void VO::VOStop()
{
    _vorunning.store(false);
    _vothread.join();
}