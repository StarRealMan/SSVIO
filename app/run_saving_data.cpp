#include "normal_include.h"
#include <pcl/io/pcd_io.h>


int main(int argc,char** argv)
{
	Xtion_Camera::Ptr xtion_cam(new Xtion_Camera);
	std::cout << "camera init ok!" << std::endl;
	MyViewer::Ptr viewer(new MyViewer(xtion_cam));
	std::cout << "viewer init ok!" << std::endl;
	int saving_data_num = 0;

	while(viewer->getKeyVal() != 27 )
	{
		if(viewer->getKeyVal() == 't')
		{
			cv::imwrite("../savings/rgb" + std::to_string(saving_data_num) + ".jpg", xtion_cam->getRGBImage());
			cv::imwrite("../savings/depth" + std::to_string(saving_data_num) + ".jpg", xtion_cam->getDImage());
			xtion_cam->getRGB3DPoint();
		}
	}
	
	viewer->ViewerStop();
	std::cout << "viewer closed!" << std::endl;
	xtion_cam->GrabStop();
	std::cout << "camera closed!" << std::endl;

	return 0;
}