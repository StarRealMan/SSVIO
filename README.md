# SSVIO
Graduation Project: A point cloud semantic segmentation and VIO based 3D reconstruction method using RGB-D and IMU

**Newest result**

![avatar.png](https://github.com/StarRealMan/SSVIO/blob/main/info/coordinate.png?raw=true)

# REMEMBER!
**Note that the metric of pointcloud is same as the LSB of depth sensor!**

**The coordinate of camera and world is as follow(Red:X, Blue:Y, Green:Z):**

![reconstruction.png](https://github.com/StarRealMan/SSVIO/blob/main/info/reconstruction.png?raw=true)

## Requirement
* gcc and cmake
* OpenCV:<https://opencv.org/>
* PCL:<https://pointclouds.org/>
* g2o:<https://github.com/RainerKuemmerle/g2o>
* Sophus:<https://github.com/strasdat/Sophus>
* Eigen:<http://eigen.tuxfamily.org/index.php?title=Main_Page>
* Pangolin:<https://github.com/stevenlovegrove/Pangolin>
* suitesparse:
use
```
apt install libsuitesparse-dev
```
* openni:
use 
```
apt install libopenni2-dev
```

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
* visual reconstruction app:
```
    ./run_visual_reconstruct
```
map pointcloud data will be saved at {YOUR_DIRECTORY}/savings/map.pcd

### Saving Data
* photo shooting and data saving app:
```
    ./run_saving_data
```
Press "t" to take shot and save at {YOUR_DIRECTORY}/savings

* Run at {YOUR_DIRECTORY}
```
    bash ./removedata.sh
```
to remove all the saved files

### Feature matching Pose optimization and Frame jointment
* feature matching app:
```
    ./run_feature_match
```
input saved data num * 2

output matched feature points

* pose optimization:
```
    ./run_g2o_optim
```
input saved data num * 2

output pose and matched feature points

* frame jointment:
```
    ./run_frame_jointment
```
input saved data num * 2

output frame_joint.pcd saved at  {YOUR_DIRECTORY}/savings/pointcloud

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

![avatar.png](https://github.com/StarRealMan/SSVIO/blob/main/info/avatar.png?raw=true)

Student from HITSZ Automatic Control NRS-lab