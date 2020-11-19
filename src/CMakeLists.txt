add_library(visual_recon SHARED
            Reconstruct.cpp
            Visual_Odometry.cpp
            Frame.cpp
            # Map.cpp
            Viewer.cpp)

target_link_libraries(visual_recon
                      xtion_driver
                      ${THIRD_PARTY_LIBS})