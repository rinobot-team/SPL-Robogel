project(GameController CXX)

set(srcs
    RoboCupGameControlData.hpp
    RoboCupGameControlData.cpp
    GameController.hpp
    GameController.cpp
)
add_library(${PROJECT_NAME} ${srcs})
# target_include_directories(utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/utils)
# target_link_libraries(${PROJECT_NAME} cpp17)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17 LINKER_LANGUAGE CXX)
