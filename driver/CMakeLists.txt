add_library(xtion_driver SHARED
            Xtion_Driver.cpp)
            
add_library(xtion2_driver SHARED
            Xtion2_Driver.cpp)


target_link_libraries(xtion_driver
                      ${THIRD_PARTY_LIBS})

target_link_libraries(xtion2_driver
                      ${THIRD_PARTY_LIBS})