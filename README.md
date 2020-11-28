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

### Visual Reconstruction
visual reconstruction app:
```
    ./run_visual_reconstruct
```
### Saving Data
photo shooting and data saving
```
    ./run_saving_data
```
Press "t" to take photo and save at ./savings
Run  
```
    bash ./removall.sh
```
to remove all the saved files

## Visualization
After running ./run_visual_reconstruct, you will see three windows:
* RGB for RGB image
* Depth for depth image
* PointCloud for PointCloud viewer

Press "Esc" at RGB window or press "Ctrl-C" at terminal to stop the program

## Bug
* Sometimes go into error with code "segmentation fault"
* Calculating the corresponding point to be at the same place
* Finding the feature point at (0,0,0)

## Author

![avatar.png](https://github.com/StarRealMan/SSVIO/blob/main/avatar.png?raw=true)

Student from HITSZ Automatic Control NRS-lab