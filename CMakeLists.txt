cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)

set(PROJECT_NAME HandReacher)
set(PROJECT_DESCRIPTION "mc-rtc controller for HandReacher task using RL policy.")
set(PROJECT_URL "https://github.com/rohanpsingh/mc_reacher_policy")

project(${PROJECT_NAME} CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

if(NOT EXISTS ${PROJECT_SOURCE_DIR}/ext/libtorch)
  # FetchContent from CMake >= 3.11
  if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11")
    message(STATUS "libtorch is missing. Will be automatically downloaded.")
    include(FetchContent)
    FetchContent_Declare(libtorch
      URL https://download.pytorch.org/libtorch/nightly/cpu/libtorch-cxx11-abi-shared-with-deps-latest.zip
      SOURCE_DIR ${PROJECT_SOURCE_DIR}/ext/libtorch
      )
    FetchContent_Populate(libtorch)
  else()
    message(FATAL_ERROR "libtorch is missing at ${PROJECT_SOURCE_DIR}/ext/libtorch. Download from https://download.pytorch.org/libtorch/nightly/cpu/libtorch-cxx11-abi-shared-with-deps-latest.zip")
  endif()
endif()

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} ${PROJECT_SOURCE_DIR}/ext/libtorch)

# Check if the project is built inside mc_rtc
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/HandReacher.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/HandReacher.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/HandReacher.yaml" DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
install(FILES "${PROJECT_SOURCE_DIR}/models/actor.pt" DESTINATION "/tmp/")

