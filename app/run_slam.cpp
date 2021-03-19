#include "Init.h"
#include "Xtion_Driver.h"

int main(int argc, char** argv)
{
    XtionCamera::Ptr camera(new XtionCamera(config));
	cout << "Camera Init OK!" << endl;
    Map::Ptr map(new Map());
	cout << "Map Init OK!" << endl;
    IMU::Ptr imu(new IMU(config));
	cout << "IMU Init OK!" << endl;
    Odometry::Ptr odometry(new Odometry(camera, imu, map, config));
	cout << "Odometry Init OK!" << endl;
    Local::Ptr local(new Local(map, config));
	cout << "Local Init OK!" << endl;
    Viewer::Ptr viewer(new Viewer(camera, odometry));
	cout << "Viewer Init OK!" << endl;

    while(viewer->GetKeyVal() != 27)
    {
        // do something
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

	viewer->ViewerStop();
	std::cout << "Viewer closed!" << std::endl;
    local->LocalStop();
    std::cout << "Local closed!" << std::endl;
    odometry->OdometryStop();
    std::cout << "Odometry closed!" << std::endl;
    imu->IMUStop();
	std::cout << "IMU closed!" << std::endl;
    camera->CameraStop();
	std::cout << "Camera closed!" << std::endl;

    return 0;

}