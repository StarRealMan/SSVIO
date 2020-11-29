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
photo shooting and data saving app:
```
    ./run_saving_data
```
Press "t" to take photo and save at ./savings

Run at project directory
```
    bash ./removedata.sh
```
to remove all the saved files

### Feature matching and Pose optimization
feature matching app:
```
    ./run_feature_match
```
input saved data num * 2

output matched feature points

pose optimization:
```
    ./run_g2o_optim
```
input saved data num * 2

output pose and matched feature points

## Visualization
After running ./run_visual_reconstruct, you will see three windows:
* RGB for RGB image
* Depth for depth image
* PointCloud for PointCloud viewer

Press "Esc" at RGB window or press "Ctrl-C" at terminal to stop the program

## Bug
* Sometimes image grabbed has error (maybe lock error ?)
* New method to pick good match points
* Poor match points need to be kicked while optimizing dynamicaly



## Author

![avatar.png](https://github.com/StarRealMan/SSVIO/blob/main/avatar.png?raw=true)

Student from HITSZ Automatic Control NRS-lab