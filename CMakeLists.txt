cmake_minimum_required(VERSION 3.5)

project(Robogel)

# set(CMAKE_TOOLCHAIN_FILE /home/vagrant/vai/cross-config.cmake CACHE)

add_subdirectory(LolaConnector)
add_subdirectory(HTWKMotion)
add_subdirectory(utils)
add_subdirectory(GameController)

add_compile_options(-O3 -Wall -march=core2 -mtune=core2 -mfpmath=sse -msse4 -ggdb)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

add_executable(${PROJECT_NAME}
main.cpp)

add_dependencies(${PROJECT_NAME} WalkingEngine)
add_dependencies(${PROJECT_NAME} lola_connector)
add_dependencies(${PROJECT_NAME} utils)
add_dependencies(${PROJECT_NAME} GameController)

target_link_libraries(${PROJECT_NAME} Boost::system msgpack )
target_link_libraries(${PROJECT_NAME} utils lola_connector WalkingEngine)
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
