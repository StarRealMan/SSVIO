#include "normal_include.h"


int main(int argc,char** argv)
{
	Config::Ptr config(new Config("../config/default_conf.yaml"));
	Xtion_Camera::Ptr xtion_cam(new Xtion_Camera);
	std::cout << "camera init ok!" << std::endl;
	MyViewer::Ptr viewer(new MyViewer(xtion_cam));
	std::cout << "viewer init ok!" << std::endl;
	VO::Ptr vo(new VO(xtion_cam,config));
	std::cout << "vo init ok!" << std::endl;
	

	while(viewer->getKeyVal() != 27 )
	{
		// thread run
		// fps calculate
	}

	vo->VOStop();
	std::cout << "vo closed!" << std::endl;
	viewer->ViewerStop();
	std::cout << "viewer closed!" << std::endl;
	xtion_cam->GrabStop();
	std::cout << "camera closed!" << std::endl;

	return 0;
}