#include "normal_include.h"
#include <pcl/io/pcd_io.h>


int main(int argc,char** argv)
{
	Xtion_Camera::Ptr xtion_cam(new Xtion_Camera);
	std::cout << "camera init ok!" << std::endl;
	MyViewer::Ptr viewer(new MyViewer(xtion_cam));
	std::cout << "viewer init ok!" << std::endl;
	VO::Ptr vo(new VO(xtion_cam));
	std::cout << "vo init ok!" << std::endl;

	while(viewer->getKeyVal() != 27 )
	{
		// thread run
		// fps calculate
	}

	vo->VOStop();
	viewer->ViewerStop();
	xtion_cam->GrabStop();

	return 0;
}