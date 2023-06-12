cmake_minimum_required(VERSION 3.5)

project(cameraAgora)

add_compile_options(-O3 -Wall -march=core2 -mtune=core2 -mfpmath=sse -msse4 -ggdb)
find_package(Boost COMPONENTS system program_options filesystem REQUIRED)

add_executable(${PROJECT_NAME}
		CameraDefinitions.hpp
		types/CameraSettings.hpp
		types/CameraSettings.cpp
		NaoCameraDefinitions.hpp
		NaoCameraDefinitions.cpp
		types/alvisiondefinitions.h
		Camera.hpp
		Camera.cpp
		NaoCamera.hpp
		NaoCamera.cpp
		utils/Timer.hpp
		utils/Timer.cpp
		main.cpp
)
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC .)
set_target_properties(${PROJECT_NAME} PROPERTIES CXX_STANDARD 14)
