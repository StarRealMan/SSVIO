add_library(xtion_driver SHARED
            Xtion_Driver.cpp)

target_link_libraries(xtion_driver
                      ${THIRD_PARTY_LIBS})