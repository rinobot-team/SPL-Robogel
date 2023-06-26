project(utils CXX)

set(srcs
    exponential_moving_average.h
    fsr.h
    imu.h
    joints.h
    point_2d.h
    point_3d.h
    position.h
    stl_ext.h
    state.h
)
add_library(${PROJECT_NAME} ${srcs})
# target_include_directories(utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/utils)
# target_link_libraries(${PROJECT_NAME} cpp17)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 LINKER_LANGUAGE CXX)
