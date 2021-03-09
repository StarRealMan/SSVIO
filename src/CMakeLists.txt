add_library(libSSVIO SHARED
            Config.cpp
            Feature.cpp
            FMatch.cpp
            Frame.cpp
            IMU.cpp
            Local.cpp
            Loop.cpp
            Map.cpp
            Odometry.cpp
            Optimizer.cpp
            Viewer.cpp
)

target_link_libraries(libSSVIO
                      xtion_driver
                      ${THIRD_PARTY_LIBS})