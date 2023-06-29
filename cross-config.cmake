set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR i686)

set(CTC_DIR_PATH "../softwares/ctc-linux64-atom-2.8.5.10")
# softwares/ctc-linux64-atom-2.8.5.10.zip

set(CMAKE_SYSROOT "${CMAKE_CURRENT_LIST_DIR}/${CTC_DIR_PATH}/yocto-sdk/sysroots/core2-32-sbr-linux")

set(tools "${CMAKE_CURRENT_LIST_DIR}/${CTC_DIR_PATH}/yocto-sdk/sysroots/x86_64-naoqisdk-linux/usr/bin/i686-sbr-linux")
set(CMAKE_C_COMPILER "${tools}/i686-sbr-linux-gcc")
set(CMAKE_CXX_COMPILER "${tools}/i686-sbr-linux-g++")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
