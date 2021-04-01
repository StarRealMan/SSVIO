#include "Init.h"

int main(int argc, char** argv)
{
    IMU::Ptr imu(new IMU(config));
    Eigen::Vector3f vec;
    Eigen::Matrix3f mat;
    sleep(1);

    while(1)
    {
        imu->GetIMURotateData(mat);
        imu->GetIMUTransitData(vec);
        std::cout << mat << std::endl;
        std::cout << vec << std::endl;
        sleep(1);
    }

    imu->IMUStop();

    return 0;
}