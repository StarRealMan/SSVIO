#include "normal_include.h"

int main(int argc,char** argv)
{
	Xtion_Camera::Ptr xtion_cam(new Xtion_Camera);
	std::cout << "camera init ok!" << std::endl;
	MyViewer::Ptr viewer(new MyViewer(xtion_cam));
	std::cout << "viewer init ok!" << std::endl;
	int saving_data_num = 0;
  	pcl::PCDWriter Pclwriter;

	while(viewer->getKeyVal() != 27 )
	{
		if(viewer->getKeyVal() == 's')
		{
			cv::imwrite("../savings/rgb/rgb" + std::to_string(saving_data_num) + ".jpg", xtion_cam->getRGBImage());
			cv::imwrite("../savings/depth/depth" + std::to_string(saving_data_num) + ".jpg", xtion_cam->getDImage());
  			Pclwriter.write("../savings/pointcloud/pointcloud" + std::to_string(saving_data_num) + ".pcd",*(xtion_cam->getRGBCloud()));
			saving_data_num++;
		}
	}
	
	viewer->ViewerStop();
	std::cout << "viewer closed!" << std::endl;
	xtion_cam->GrabStop();
	std::cout << "camera closed!" << std::endl;

	return 0;
}