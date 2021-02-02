#include "Xtion2_Driver.h"
#include <boost/make_shared.hpp>

Xtion2_Camera::Xtion2_Camera()
{
    rc = openni::STATUS_OK;
	rc = openni::OpenNI::initialize();
    if(rc != openni::STATUS_OK)
    {
        std::cout << "openni初始化失败" << std::endl;
        closecamera();
        exit(0);
    }
    openni::OpenNI::enumerateDevices(&_deviceList);
    if(_deviceList.getSize() < 1)
    {
        std::cout << "没有设备连接！" << std::endl;
        closecamera();
        exit(0);
    }

    showdevice();
    std::string uri = "";
    vendor = "ASUS";
    for(ushort i = 0; i < _deviceList.getSize(); ++i)
    {
        const openni::DeviceInfo &rDevInfo = _deviceList[i];
        if(vendor == rDevInfo.getVendor())
        {
            uri = rDevInfo.getUri();
            break;
        }
    }
    if(uri.empty())
    {
        std::cout << "没有Xtion2连接！" << std::endl;
    }
	rc = _camera.open(uri.c_str());
 
	rc = _streamDepth.create(_camera, openni::SENSOR_DEPTH);
	if( rc == openni::STATUS_OK)
	{
		_modeD.setResolution(640,480);
		_modeD.setFps(30);
		_modeD.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
		_streamDepth.setVideoMode(_modeD);
 
		_streamDepth.start();
		if(rc != openni::STATUS_OK)
		{
			std::cerr << "无法打开深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
			_streamDepth.destroy();
		}
	}
	else
	{
		std::cerr << "无法创建深度数据流："<< openni::OpenNI::getExtendedError() << std::endl;
	}

	rc = _streamRGB.create(_camera, openni::SENSOR_COLOR);
	if(rc == openni::STATUS_OK)
	{
		_modeRGB.setResolution(640,480);
		_modeRGB.setFps(30);
		_modeRGB.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
 
		if(_camera.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			_camera.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
		}
 
		rc = _streamRGB.start();
		if( rc != openni::STATUS_OK)
		{
			std::cerr << "无法打开彩色数据流："<< openni::OpenNI::getExtendedError() << std::endl;
			_streamRGB.destroy();
		}
	}
	else
	{
		std::cerr << "无法创建彩色数据流："<< openni::OpenNI::getExtendedError() << std::endl;
	}
 
	if(!_streamDepth.isValid() || !_streamRGB.isValid())
	{
		std::cerr << "数据流不合法" << std::endl;
		openni::OpenNI::shutdown();
		exit(0);
	}

    _height = 480;
    _width = 640;
    _inner_cx = 160.5912;
    _inner_cy = 120.4792;
    _inner_fx = 253.0589;
    _inner_fy = 254.1649;
    _inv_inner_fx = 1/_inner_fx;
    _inv_inner_fy = 1/_inner_fy;

    _rgbCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    _rgbCloud->width = _height * _width;
    _rgbCloud->height = 1;
    _rgbCloud->points.resize(_rgbCloud->width * _rgbCloud->height);

    _grabrunning.store(true);
    _grabthread = std::thread(std::bind(&Xtion2_Camera::GrabLoop,this));
}

Xtion2_Camera::~Xtion2_Camera()
{
    closecamera();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Xtion2_Camera::getRGBCloud()
{
    std::lock_guard<std::mutex> lck(_rgbcloud_mtx);
    return _rgbCloud;
}

cv::Mat Xtion2_Camera::getDImage()
{
    std::lock_guard<std::mutex> lck(_dimage_mtx);
    return _dImage;
}

cv::Mat Xtion2_Camera::getRGBImage()
{
    std::lock_guard<std::mutex> lck(_rgbimage_mtx);
    return _rgbImage;
}

pcl::PointXYZRGB Xtion2_Camera::getRGB3DPoint(int pos_x, int pos_y, const Eigen::Matrix4f& trans)
{        
    std::lock_guard<std::mutex> lck_d(_dimage_mtx);
    std::lock_guard<std::mutex> lck_rgb(_rgbimage_mtx);
    Eigen::Vector4f temp;
    pcl::PointXYZRGB rgb3dpoint;

    ushort depth = _dImage.at<ushort>(pos_y,pos_x);

    temp << (pos_x - _inner_cx)*depth*_inv_inner_fx,
            (pos_y - _inner_cy)*depth*_inv_inner_fy,
            depth, 1.0;

    temp = trans * temp;
    rgb3dpoint.x = temp[0];
    rgb3dpoint.y = temp[1];
    rgb3dpoint.z = temp[2];
    // 注意尺度单位为深度传感器LSB
    rgb3dpoint.b = _rgbImage.at<cv::Vec3b>(pos_y,pos_x)[0];
    rgb3dpoint.g = _rgbImage.at<cv::Vec3b>(pos_y,pos_x)[1];
    rgb3dpoint.r = _rgbImage.at<cv::Vec3b>(pos_y,pos_x)[2];

    return rgb3dpoint;
}

bool Xtion2_Camera::isGrabRdy()
{
    std::lock_guard<std::mutex> lck(_grabrdy_mtx);
    return _grabRdy;
}

void Xtion2_Camera::setGrabRdyfalse()
{
    std::lock_guard<std::mutex> lck(_grabrdy_mtx);
    _grabRdy = false;
}

void Xtion2_Camera::showdevice()
{
    std::cout << "电脑上连接着 " << _deviceList.getSize() << " 个体感设备." << std::endl;
    for(uchar i = 0; i < _deviceList.getSize(); ++i)
    {
        std::cout << "设备 " << i << std::endl;
        const openni::DeviceInfo &rDevInfo = _deviceList[i];
        std::cout << "设备名： " << rDevInfo.getName() << std::endl;
        std::cout << "设备Id： " << rDevInfo.getUsbProductId() << std::endl;
        std::cout << "供应商名： " << rDevInfo.getVendor() << std::endl;
        std::cout << "供应商Id: " << rDevInfo.getUsbVendorId() << std::endl;
        std::cout << "设备URI: " << rDevInfo.getUri() << std::endl;
    }
}

void Xtion2_Camera::closecamera()
{
    _streamDepth.destroy();
    _streamRGB.destroy();
    _camera.close();
    openni::OpenNI::shutdown();
}

void Xtion2_Camera::grab()
{
    bool Depth_OK = false, RGB_OK = false;

    if( openni::STATUS_OK == _streamDepth.readFrame(&_onidImage) )
    {
        std::lock_guard<std::mutex> lck(_dimage_mtx);
        cv::Mat cvRawImg16U(_onidImage.getHeight(), _onidImage.getWidth(), CV_16UC1, (void*)_onidImage.getData());
        cvRawImg16U.convertTo(_dImage, CV_16U, 65535.0/(_streamDepth.getMaxPixelValue()));
        cv::flip(_dImage,_dImage,1);
        Depth_OK = true;
    }
    if( openni::STATUS_OK == _streamRGB.readFrame(&_onirgbImage) )
    {
        std::lock_guard<std::mutex> lck(_rgbimage_mtx);
        cv::Mat cvRGBImg(_onirgbImage.getHeight(), _onirgbImage.getWidth(), CV_8UC3, (void*)_onirgbImage.getData());
        cv::cvtColor(cvRGBImg, _rgbImage, CV_RGB2BGR);
        cv::flip(_rgbImage,_rgbImage,1);
        RGB_OK = true;
    }
    if(Depth_OK && RGB_OK)
    {
        for(ushort u = 0; u < _dImage.cols; u++)
        {
            for(ushort v = 0; v < _dImage.rows; v++)
            {
                int i = u * _dImage.rows + v;
                _rgbCloud->points[i] = getRGB3DPoint(u,v,Eigen::Matrix4f::Identity());
            }
        }
    }
    if(Depth_OK && RGB_OK)
    {
        std::lock_guard<std::mutex> lck_rdy(_grabrdy_mtx);
        _grabRdy = true;
        Depth_OK = false;
        RGB_OK = false;
    }

}

void Xtion2_Camera::GrabLoop()
{
    while(_grabrunning.load())
    {
        grab();
    }
}

void Xtion2_Camera::GrabStop()
{
    _grabrunning.store(false);
    _grabthread.join();
}

