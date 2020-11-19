add_executable(run_visual_reconstruct
               run_visual_reconstruct.cpp)

target_link_libraries(run_visual_reconstruct
                      xtion_driver
                      visual_recon
                      ${THIRD_PARTY_LIBS})