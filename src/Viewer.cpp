#include "Viewer.h"

Viewer::Viewer(XtionCamera::Ptr camera, Odometry::Ptr odometry):_pcl_viewer("Simple Cloud Viewer")
{
 	cv::namedWindow("Depth",cv::WINDOW_NORMAL);
	cv::namedWindow("RGB",cv::WINDOW_NORMAL);
    
    _viewer_cam = camera;
    _viewer_odometry = odometry;
    
    _viewer_running.store(true);
    _viewer_thread = std::thread(std::bind(&Viewer::ViewerLoop,this));
}

Viewer::~Viewer()
{
	cv::destroyWindow("Depth");
	cv::destroyWindow("RGB");
}

char Viewer::GetKeyVal()
{
    std::lock_guard<std::mutex> lck(_key_val_mtx);
    return _key_val;
}

void Viewer::ViewerLoop()
{   
    cv::Mat out_img;

    while(_viewer_running.load())
    {
        auto t1 = std::chrono::steady_clock::now();
     
        _cur_frame = _viewer_odometry->GetCurFrame();
        if(!(_cur_frame->GetRGBImage().empty()))
        {
            cv::drawKeypoints(_cur_frame->GetRGBImage(), _cur_frame->GetKeyPoints(), out_img);
            cv::imshow("RGB", out_img);
            cv::imshow("Depth", _cur_frame->GetDImage());
            cv::imshow("Depth", _cur_frame->GetDImage());
            _pcl_viewer.showCloud(_viewer_cam->GetRGBCloud());
            _key_val = cv::waitKey(31);
        }

        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Viewer Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void Viewer::ViewerStop()
{
    _viewer_running.store(false);
    _viewer_thread.join();
}
