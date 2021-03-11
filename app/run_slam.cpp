#include "Odometry.h"
#include "Local.h"
#include "Loop.h"
#include "Viewer.h"
#include "Xtion_Driver.h"
#include "Config.h"
#include "Frame.h"
#include "MapPoint.h"
#include "Local.h"

Config::Ptr config(new Config("../config/default_conf.yaml"));

float Frame::_InnerCx = config->GetParam<float>("InnerCx");
float Frame::_InnerCy = config->GetParam<float>("InnerCy");
float Frame::_InnerFx = config->GetParam<float>("InnerFx");
float Frame::_InnerFy = config->GetParam<float>("InnerFy");
int Frame::_MaxGoodPointThres = config->GetParam<int>("MaxGoodPointThres");
int Frame::_MaxFramesBetween = config->GetParam<int>("MaxFramesBetween");
int Frame::_MinFramesBetween = config->GetParam<int>("MinFramesBetween");
double Frame::_DepthScale = config->GetParam<float>("DepthScale")/65535.0;
float Frame::_InvInnerFx = 1/Frame::_InnerFx;
float Frame::_InvInnerFy = 1/Frame::_InnerFy;

int Frame::_key_frame_point_num = 0;
int MapPoint::_map_point_num = 0;

int main(int argc, char** argv)
{
    XtionCamera::Ptr camera(new XtionCamera(config));
	cout << "Camera Init OK!" << endl;
    Map::Ptr map(new Map());
	cout << "Map Init OK!" << endl;
    Odometry::Ptr odometry(new Odometry(camera, map, config));
	cout << "Odometry Init OK!" << endl;
    Local::Ptr local(new Local(map, config));
	cout << "Local Init OK!" << endl;
    Viewer::Ptr viewer(new Viewer(camera, odometry));
	cout << "Viewer Init OK!" << endl;

    while(viewer->GetKeyVal() != 27)
    {

    }

	viewer->ViewerStop();
	std::cout << "Viewer closed!" << std::endl;
    local->LocalStop();
    std::cout << "Local closed!" << std::endl;
    odometry->OdometryStop();
    std::cout << "Odometry closed!" << std::endl;
	camera->CameraStop();
	std::cout << "Camera closed!" << std::endl;

    return 0;

}