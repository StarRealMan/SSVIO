# SSVIO
Graduation Project: A point cloud semantic segmentation and VIO based 3D reconstruction method using RGB-D and IMU

## Requirement
* gcc and cmake
* OpenCV
* PCL
* g2o
* Sophus
* Eigen
* Pangolin

## Usage
* Go to {YOUR_DIRECTORY}/
* Run following code
```
    mkdir build
    cd ./build
    cmake ..
    make
    sudo make install
```
* After generating bin file, go to {YOUR_DIRECTORY}/bin
* Run following code
    ./run_visual_reconstruct

## Visualization
After running ./run_visual_reconstruct, you will see three windows:
* RGB for RGB image
* Depth for depth image
* PointCloud for PointCloud viewer

Press "Esc" at RGB window or press ctrl-c at terminal to stop the program

## Bug
* Sometimes go into error with code "segmentation fault"
* Calculating the corresponding point to be at the same place
* Finding the feature point at (0,0,0)