add_executable(run_visual_reconstruct
               run_visual_reconstruct.cpp)

target_link_libraries(run_visual_reconstruct
                      xtion_driver
                      visual_recon
                      ${THIRD_PARTY_LIBS})


file(MAKE_DIRECTORY "../savings")
file(MAKE_DIRECTORY "../savings/rgb")
file(MAKE_DIRECTORY "../savings/depth")
file(MAKE_DIRECTORY "../savings/pointcloud")

add_executable(run_saving_data
               run_saving_data.cpp)

target_link_libraries(run_saving_data
                      xtion_driver
                      visual_recon
                      ${THIRD_PARTY_LIBS})
