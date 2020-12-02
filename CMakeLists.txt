cmake_minimum_required(VERSION 2.8)
project(Visual_Reconstruct)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++14 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE  "-std=c++14 -O3 -fopenmp -pthread")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

############### dependencies ######################
# Eigen
include_directories("/usr/include/eigen3")

# OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
find_Package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
set(G2O_LIBS g2o_core g2o_stuff g2o_types_sba g2o_types_slam3d
             g2o_solver_csparse g2o_csparse_extension cxsparse)
             # cholmod g2o_solver_cholmod "unused"

# PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common 
                                         io
                                         features
                                         visualization
                                         filters
                                         keypoints
                                         segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

set(THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${CSPARSE_LIBRARY}
        ${G2O_LIBS}
        ${PCL_LIBRARIES}
        )

enable_testing()

################### source and driver ######################
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/driver)
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_subdirectory(driver)
add_subdirectory(src)
add_subdirectory(app)