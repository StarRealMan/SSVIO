add_executable(run_slam
               run_slam.cpp)

target_link_libraries(run_slam
                      xtion_driver
                      libSSVIO
                      ${THIRD_PARTY_LIBS})


file(MAKE_DIRECTORY "../savings")
file(MAKE_DIRECTORY "../savings/rgb")
file(MAKE_DIRECTORY "../savings/depth")
file(MAKE_DIRECTORY "../savings/pointcloud")


add_executable(run_imu
               run_imu.cpp)

target_link_libraries(run_imu
                      xtion_driver
                      libSSVIO
                      ${THIRD_PARTY_LIBS})
