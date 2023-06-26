cmake_minimum_required(VERSION 3.5)

project(lola_connector)

add_subdirectory(msgpack)

add_compile_options(-O3 -Wall -march=core2 -mtune=core2 -mfpmath=sse -msse4 -ggdb)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

add_library(${PROJECT_NAME}
    battery.h
    fila.h
    fila.cpp
    # fsr.h
    # imu.h
    # joints.h
    leds.cpp
    leds.h
    # lola_connector.cpp
    lola_connector.h
    lola_frame.cpp
    lola_frame.h
    # point_3d.h
    # stl_ext.h

)
target_link_libraries(${PROJECT_NAME} Boost::system msgpack utils)
target_include_directories(${PROJECT_NAME} PUBLIC .)
target_include_directories(${PROJECT_NAME} PUBLIC ../utils)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
