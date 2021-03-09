#include "Odometry.h"
#include "Local.h"
#include "Loop.h"
#include "Viewer.h"
#include "Xtion_Driver.h"
#include "Config.h"
#include "Frame.h"

int main(int argc, char** argv)
{
    Config::Ptr config(new Config("../config/default_conf.yaml"));
    XtionCamera::Ptr camera(new XtionCamera(config));
	cout << "Camera Init OK!" << endl;
    Odometry::Ptr odometry(new Odometry(camera, config));
	cout << "Odometry Init OK!" << endl;
    Viewer::Ptr viewer(new Viewer(camera, odometry));
	cout << "Viewer Init OK!" << endl;

    while(viewer->GetKeyVal() != 27)
    {

    }

	viewer->ViewerStop();
	std::cout << "Viewer closed!" << std::endl;
    odometry->OdometryStop();
    std::cout << "Odometry closed!" << std::endl;
	camera->CameraStop();
	std::cout << "Camera closed!" << std::endl;

    return 0;

}