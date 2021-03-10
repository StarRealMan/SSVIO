#include "Xtion_Driver.h"
#include <boost/make_shared.hpp>

XtionCamera::XtionCamera(Config::Ptr config)
{

    _UseXtionGen = config->GetParam<int>("UseXtionGen");
    _Fps = config->GetParam<int>("Fps");
    _ImgHeight = config->GetParam<int>("ImgHeight");
    _ImgWidth = config->GetParam<int>("ImgWidth");
    _InnerCx = config->GetParam<float>("InnerCx");
    _InnerCy = config->GetParam<float>("InnerCy");
    _InnerFx = config->GetParam<float>("InnerFx");
    _InnerFy = config->GetParam<float>("InnerFy");
    _DepthScale = config->GetParam<float>("DepthScale")/65535.0;
    _InvInnerFx = 1/_InnerFx;
    _InvInnerFy = 1/_InnerFy;

    rc = openni::STATUS_OK;
	rc = openni::OpenNI::initialize();
    if(rc != openni::STATUS_OK)
    {
        std::cout << "openni初始化失败" << std::endl;
        CloseCamera();
        exit(0);
    }
    openni::OpenNI::enumerateDevices(&_device_list);
    if(_device_list.getSize() < 1)
    {
        std::cout << "没有设备连接！" << std::endl;
        CloseCamera();
        exit(0);
    }

    ShowDevice();
    std::string uri = "";
    _vendor = "PrimeSense";
    for(ushort i = 0; i < _device_list.getSize(); ++i)
    {
        const openni::DeviceInfo &rDevInfo = _device_list[i];
        if(_vendor == rDevInfo.getVendor())
        {
            uri = rDevInfo.getUri();
            break;
        }
    }
    if(uri.empty())
    {
        std::cout << "没有Xtion1连接！" << std::endl;
    }
	rc = _camera.open(uri.c_str());
 
	rc = _stream_depth.create(_camera, openni::SENSOR_DEPTH);
	if( rc == openni::STATUS_OK)
	{
		_mode_d.setResolution(_ImgWidth,_ImgHeight);
		_mode_d.setFps(_Fps);
		_mode_d.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		_stream_depth.setVideoMode(_mode_d);
 
		_stream_depth.start();
		if(rc != openni::STATUS_OK)
		{
			std::cerr << "无法打开深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
			_stream_depth.destroy();
		}
	}
	else
	{
		std::cerr << "无法创建深度数据流："<< openni::OpenNI::getExtendedError() << std::endl;
	}

	rc = _stream_rgb.create(_camera, openni::SENSOR_COLOR);
	if(rc == openni::STATUS_OK)
	{
		_mode_rgb.setResolution(_ImgWidth,_ImgHeight);
		_mode_rgb.setFps(_Fps);
		_mode_rgb.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
 
		if(_camera.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			_camera.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}
 
		rc = _stream_rgb.start();
		if( rc != openni::STATUS_OK)
		{
			std::cerr << "无法打开彩色数据流："<< openni::OpenNI::getExtendedError() << std::endl;
			_stream_rgb.destroy();
		}
	}
	else
	{
		std::cerr << "无法创建彩色数据流："<< openni::OpenNI::getExtendedError() << std::endl;
	}
 
	if(!_stream_depth.isValid() || !_stream_rgb.isValid())
	{
		std::cerr << "数据流不合法" << std::endl;
		openni::OpenNI::shutdown();
		exit(0);
	}

    _ImgDepthCoe = 65535.0/(_stream_depth.getMaxPixelValue());

    _rgb_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    _rgb_cloud->width = _ImgHeight * _ImgWidth;
    _rgb_cloud->height = 1;
    _rgb_cloud->points.resize(_rgb_cloud->width * _rgb_cloud->height);

    _grab_running.store(true);
    _grab_thread = std::thread(std::bind(&XtionCamera::GrabLoop,this));
}

XtionCamera::~XtionCamera()
{
    CloseCamera();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr XtionCamera::GetRGBCloud()
{
    std::lock_guard<std::mutex> lck(_rgb_cloud_mtx);
    return _rgb_cloud;
}

cv::Mat XtionCamera::GetDImage()
{
    std::lock_guard<std::mutex> lck(_d_image_mtx);
    return _d_image;
}

cv::Mat XtionCamera::GetRGBImage()
{
    std::lock_guard<std::mutex> lck(_rgb_image_mtx);
    return _rgb_image;
}

pcl::PointXYZRGB XtionCamera::GetRGB3DPoint(int pos_x, int pos_y, const Eigen::Matrix4f& trans)
{        
    std::lock_guard<std::mutex> lck_d(_d_image_mtx);
    std::lock_guard<std::mutex> lck_rgb(_rgb_image_mtx);
    Eigen::Vector4f temp;
    pcl::PointXYZRGB rgb3dpoint;
    // 深度图尺度单位为深度传感器LSB

    ushort depth = _d_image.at<ushort>(pos_y,pos_x)*_DepthScale;
    // 转换尺度单位为m

    
    temp << (pos_x - _InnerCx)*depth*_InvInnerFx,
            (pos_y - _InnerCy)*depth*_InvInnerFy,
            depth, 1.0;

    temp = trans * temp;
    rgb3dpoint.x = temp[0];
    rgb3dpoint.y = temp[1];
    rgb3dpoint.z = temp[2];
    
    rgb3dpoint.b = _rgb_image.at<cv::Vec3b>(pos_y,pos_x)[0];
    rgb3dpoint.g = _rgb_image.at<cv::Vec3b>(pos_y,pos_x)[1];
    rgb3dpoint.r = _rgb_image.at<cv::Vec3b>(pos_y,pos_x)[2];

    return rgb3dpoint;
}

bool XtionCamera::IsGrabRdy()
{
    std::lock_guard<std::mutex> lck(_grab_rdy_mtx);
    return _grab_rdy;
}

void XtionCamera::SetGrabRdyfalse()
{
    std::lock_guard<std::mutex> lck(_grab_rdy_mtx);
    _grab_rdy = false;
}

void XtionCamera::ShowDevice()
{
    std::cout << "电脑上连接着 " << _device_list.getSize() << " 个体感设备." << std::endl;
    for(uchar i = 0; i < _device_list.getSize(); ++i)
    {
        std::cout << "设备 " << i << std::endl;
        const openni::DeviceInfo &rDevInfo = _device_list[i];
        std::cout << "设备名： " << rDevInfo.getName() << std::endl;
        std::cout << "设备Id： " << rDevInfo.getUsbProductId() << std::endl;
        std::cout << "供应商名： " << rDevInfo.getVendor() << std::endl;
        std::cout << "供应商Id: " << rDevInfo.getUsbVendorId() << std::endl;
        std::cout << "设备URI: " << rDevInfo.getUri() << std::endl;
    }
}

void XtionCamera::CloseCamera()
{
    _stream_depth.destroy();
    _stream_rgb.destroy();
    _camera.close();
    openni::OpenNI::shutdown();
}

void XtionCamera::GrabImage()
{
    bool depth_ok = false, rgb_ok = false;

    if( openni::STATUS_OK == _stream_depth.readFrame(&_oni_d_image) )
    {
        std::lock_guard<std::mutex> lck(_d_image_mtx);
        cv::Mat cvRawImg16U(_oni_d_image.getHeight(), _oni_d_image.getWidth(), CV_16UC1, (void*)_oni_d_image.getData());
        cvRawImg16U.convertTo(_d_image, CV_16U, _ImgDepthCoe);
        cv::flip(_d_image,_d_image,1);
        depth_ok = true;
    }
    if( openni::STATUS_OK == _stream_rgb.readFrame(&_oni_rgb_image) )
    {
        std::lock_guard<std::mutex> lck(_rgb_image_mtx);
        cv::Mat cvRGBImg(_oni_rgb_image.getHeight(), _oni_rgb_image.getWidth(), CV_8UC3, (void*)_oni_rgb_image.getData());
        cv::cvtColor(cvRGBImg, _rgb_image, CV_RGB2BGR);
        cv::flip(_rgb_image,_rgb_image,1);
        rgb_ok = true;
    }
    if(depth_ok && rgb_ok)
    {
        for(ushort u = 0; u < _d_image.cols; u++)
        {
            for(ushort v = 0; v < _d_image.rows; v++)
            {
                int i = u * _d_image.rows + v;
                _rgb_cloud->points[i] = GetRGB3DPoint(u,v,Eigen::Matrix4f::Identity());
            }
        }
    }
    if(depth_ok && rgb_ok)
    {
        std::lock_guard<std::mutex> lck_rdy(_grab_rdy_mtx);
        _grab_rdy = true;
        depth_ok = false;
        rgb_ok = false;
    }

}

void XtionCamera::GrabLoop()
{
    while(_grab_running.load())
    {
        auto t1 = std::chrono::steady_clock::now();
        GrabImage();
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        std::cout << "Grab Thread " << time_used.count()*1000 << " ms per frame " << std::endl;
    }
}

void XtionCamera::CameraStop()
{
    _grab_running.store(false);
    _grab_thread.join();
}

