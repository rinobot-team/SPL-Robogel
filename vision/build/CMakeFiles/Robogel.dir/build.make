# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vagrant/SPL-Robogel/vision

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vagrant/SPL-Robogel/vision/build

# Include any dependencies generated for this target.
include CMakeFiles/Robogel.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Robogel.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Robogel.dir/flags.make

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o: ../types/CameraSettings.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o -c /home/vagrant/SPL-Robogel/vision/types/CameraSettings.cpp

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/types/CameraSettings.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/types/CameraSettings.cpp > CMakeFiles/Robogel.dir/types/CameraSettings.cpp.i

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/types/CameraSettings.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/types/CameraSettings.cpp -o CMakeFiles/Robogel.dir/types/CameraSettings.cpp.s

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.requires

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.provides: CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.provides

CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.provides.build: CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o


CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o: ../camera/NaoCameraDefinitions.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o -c /home/vagrant/SPL-Robogel/vision/camera/NaoCameraDefinitions.cpp

CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/camera/NaoCameraDefinitions.cpp > CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.i

CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/camera/NaoCameraDefinitions.cpp -o CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.s

CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.requires

CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.provides: CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.provides

CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.provides.build: CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o


CMakeFiles/Robogel.dir/camera/Camera.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/camera/Camera.cpp.o: ../camera/Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Robogel.dir/camera/Camera.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/camera/Camera.cpp.o -c /home/vagrant/SPL-Robogel/vision/camera/Camera.cpp

CMakeFiles/Robogel.dir/camera/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/camera/Camera.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/camera/Camera.cpp > CMakeFiles/Robogel.dir/camera/Camera.cpp.i

CMakeFiles/Robogel.dir/camera/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/camera/Camera.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/camera/Camera.cpp -o CMakeFiles/Robogel.dir/camera/Camera.cpp.s

CMakeFiles/Robogel.dir/camera/Camera.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/camera/Camera.cpp.o.requires

CMakeFiles/Robogel.dir/camera/Camera.cpp.o.provides: CMakeFiles/Robogel.dir/camera/Camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/camera/Camera.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/camera/Camera.cpp.o.provides

CMakeFiles/Robogel.dir/camera/Camera.cpp.o.provides.build: CMakeFiles/Robogel.dir/camera/Camera.cpp.o


CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o: ../camera/NaoCamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o -c /home/vagrant/SPL-Robogel/vision/camera/NaoCamera.cpp

CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/camera/NaoCamera.cpp > CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.i

CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/camera/NaoCamera.cpp -o CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.s

CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.requires

CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.provides: CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.provides

CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.provides.build: CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o


CMakeFiles/Robogel.dir/utils/Timer.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/utils/Timer.cpp.o: ../utils/Timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Robogel.dir/utils/Timer.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/utils/Timer.cpp.o -c /home/vagrant/SPL-Robogel/vision/utils/Timer.cpp

CMakeFiles/Robogel.dir/utils/Timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/utils/Timer.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/utils/Timer.cpp > CMakeFiles/Robogel.dir/utils/Timer.cpp.i

CMakeFiles/Robogel.dir/utils/Timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/utils/Timer.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/utils/Timer.cpp -o CMakeFiles/Robogel.dir/utils/Timer.cpp.s

CMakeFiles/Robogel.dir/utils/Timer.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/utils/Timer.cpp.o.requires

CMakeFiles/Robogel.dir/utils/Timer.cpp.o.provides: CMakeFiles/Robogel.dir/utils/Timer.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/utils/Timer.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/utils/Timer.cpp.o.provides

CMakeFiles/Robogel.dir/utils/Timer.cpp.o.provides.build: CMakeFiles/Robogel.dir/utils/Timer.cpp.o


CMakeFiles/Robogel.dir/main.cpp.o: CMakeFiles/Robogel.dir/flags.make
CMakeFiles/Robogel.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Robogel.dir/main.cpp.o"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Robogel.dir/main.cpp.o -c /home/vagrant/SPL-Robogel/vision/main.cpp

CMakeFiles/Robogel.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Robogel.dir/main.cpp.i"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vagrant/SPL-Robogel/vision/main.cpp > CMakeFiles/Robogel.dir/main.cpp.i

CMakeFiles/Robogel.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Robogel.dir/main.cpp.s"
	/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux/i686-sbr-linux-g++ --sysroot=/home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vagrant/SPL-Robogel/vision/main.cpp -o CMakeFiles/Robogel.dir/main.cpp.s

CMakeFiles/Robogel.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Robogel.dir/main.cpp.o.requires

CMakeFiles/Robogel.dir/main.cpp.o.provides: CMakeFiles/Robogel.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Robogel.dir/build.make CMakeFiles/Robogel.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Robogel.dir/main.cpp.o.provides

CMakeFiles/Robogel.dir/main.cpp.o.provides.build: CMakeFiles/Robogel.dir/main.cpp.o


# Object files for target Robogel
Robogel_OBJECTS = \
"CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o" \
"CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o" \
"CMakeFiles/Robogel.dir/camera/Camera.cpp.o" \
"CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o" \
"CMakeFiles/Robogel.dir/utils/Timer.cpp.o" \
"CMakeFiles/Robogel.dir/main.cpp.o"

# External object files for target Robogel
Robogel_EXTERNAL_OBJECTS =

Robogel: CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o
Robogel: CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o
Robogel: CMakeFiles/Robogel.dir/camera/Camera.cpp.o
Robogel: CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o
Robogel: CMakeFiles/Robogel.dir/utils/Timer.cpp.o
Robogel: CMakeFiles/Robogel.dir/main.cpp.o
Robogel: CMakeFiles/Robogel.dir/build.make
Robogel: /home/vagrant/SPL-Robogel/softwares/ctc-linux64-atom-2.8.5.10/yocto-sdk/sysroots/core2-32-sbr-linux/usr/lib/libboost_system-mt.so
Robogel: CMakeFiles/Robogel.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vagrant/SPL-Robogel/vision/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable Robogel"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Robogel.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Robogel.dir/build: Robogel

.PHONY : CMakeFiles/Robogel.dir/build

CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/types/CameraSettings.cpp.o.requires
CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/camera/NaoCameraDefinitions.cpp.o.requires
CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/camera/Camera.cpp.o.requires
CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/camera/NaoCamera.cpp.o.requires
CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/utils/Timer.cpp.o.requires
CMakeFiles/Robogel.dir/requires: CMakeFiles/Robogel.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Robogel.dir/requires

CMakeFiles/Robogel.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Robogel.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Robogel.dir/clean

CMakeFiles/Robogel.dir/depend:
	cd /home/vagrant/SPL-Robogel/vision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vagrant/SPL-Robogel/vision /home/vagrant/SPL-Robogel/vision /home/vagrant/SPL-Robogel/vision/build /home/vagrant/SPL-Robogel/vision/build /home/vagrant/SPL-Robogel/vision/build/CMakeFiles/Robogel.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Robogel.dir/depend

