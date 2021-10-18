cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME HandReacher)
set(PROJECT_DESCRIPTION "HandReacher")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(3rd_party_ros REQUIRED COMPONENTS roscpp ros_ml_controller)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/HandReacher.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HandReacher.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HandReacher.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")

