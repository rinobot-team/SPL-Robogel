cmake_minimum_required(VERSION 3.5)

project(Robogel-Vision)

# set(CMAKE_TOOLCHAIN_FILE /home/vagrant/vai/cross-config.cmake CACHE)

add_compile_options(-O3 -Wall -march=core2 -mtune=core2 -mfpmath=sse -msse4 -ggdb)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

add_executable(${PROJECT_NAME}
		camera/CameraDefinitions.hpp
		types/CameraSettings.hpp
		types/CameraSettings.cpp
		camera/NaoCameraDefinitions.hpp
		camera/NaoCameraDefinitions.cpp
		types/alvisiondefinitions.h
		camera/Camera.hpp
		camera/Camera.cpp
		camera/NaoCamera.hpp
		camera/NaoCamera.cpp
		utils/Timer.hpp
		utils/Timer.cpp
		main.cpp
		lib/lodepng.hpp
		lib/lodepng.cpp
		src/yuvToRGBA.hpp
		src/yuvToRGBA.cpp
		src/rgbaToPNG.hpp
		src/rgbaToPNG.cpp
)
target_link_libraries(${PROJECT_NAME} Boost::system) 
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 17)
