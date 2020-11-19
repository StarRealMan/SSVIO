#include "Viewer.h"

MyViewer::MyViewer(Xtion_Camera::Ptr camera):_pclviewer("Simple Cloud Viewer")
{
 	cv::namedWindow("Depth",cv::WINDOW_NORMAL);
	cv::namedWindow("RGB",cv::WINDOW_NORMAL);

    _viewerCam = camera;
    
    _viewerrunning.store(true);
    _viewerthread = std::thread(std::bind(&MyViewer::ViewerLoop,this));
}

MyViewer::~MyViewer()
{
	cv::destroyWindow("Depth");
	cv::destroyWindow("RGB");
}

char MyViewer::getKeyVal()
{
    std::lock_guard<std::mutex> lck(_keyval_mtx);
    return _keyVal;
}

void MyViewer::ViewerLoop()
{   
    while(_viewerrunning.load())
    {
        if(_viewerCam->isGrabRdy())// 线程同步问题
        {
            std::cout << " view start " << std::endl;
            cv::imshow("RGB", _viewerCam->getRGBImage());
            cv::imshow("Depth", _viewerCam->getDImage());
            _pclviewer.showCloud(_viewerCam->getRGBCloud());
            _keyVal = cv::waitKey(33);
            std::cout << " view ok " << std::endl;
        }
    }
}

void MyViewer::ViewerStop()
{
    _viewerrunning.store(false);
    _viewerthread.join();
}
