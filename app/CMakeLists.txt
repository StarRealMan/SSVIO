add_executable(run_visual_reconstruct
               run_visual_reconstruct.cpp)

add_executable(run_saving_data
               run_saving_data.cpp)

target_link_libraries(run_visual_reconstruct
                      xtion_driver
                      visual_recon
                      ${THIRD_PARTY_LIBS})