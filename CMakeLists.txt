cmake_minimum_required(VERSION 3.25)

include(CMakePrintHelpers)
include(FetchContent)

FetchContent_Declare(
  rio_toolchain
  URL  https://github.com/wpilibsuite/opensdk/releases/download/v2023-8/cortexa9_vfpv3-roborio-academic-2023-x86_64-w64-mingw32-Toolchain-12.1.0.zip
)
FetchContent_MakeAvailable(rio_toolchain)

project(rio_compile_ex)

set(CMAKE_CXX_STANDARD 20)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake/")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(opencv REQUIRED)
find_package(cscore REQUIRED)
find_package(ntcore REQUIRED)
find_package(cameraserver REQUIRED)
find_package(wpimath REQUIRED)
find_package(apriltaglib REQUIRED)
find_package(apriltag REQUIRED)
find_package(hal REQUIRED)
find_package(wpilibc REQUIRED)
find_package(wpilibnewcommands REQUIRED)
find_package(kauailabs REQUIRED)
find_package(pathplanner REQUIRED)
find_package(photonvision REQUIRED)
find_package(rev REQUIRED)
find_package(phoenix REQUIRED)
find_package(Threads REQUIRED)

if(TOOLCHAIN_TRIPLE STREQUAL "arm-nilrt-linux-gnueabi")
  find_package(nilibraries REQUIRED)
endif()

add_executable(test_exec src/main.cpp)

if(MSVC)
  target_compile_options(test_exec PRIVATE /Zc:preprocessor /bigobj)
endif()

target_link_libraries(test_exec 
  Threads::Threads 
  phoenix
  rev
  photonvision::photonvision
  pathplanner::pathplanner
  kauailabs::kauailabs
  wpilibnewcommands::wpilibnewcommands 
  wpilibc::wpilibc 
  hal::hal 
  apriltag::apriltag 
  apriltaglib::apriltaglib 
  wpimath::wpimath 
  cameraserver::cameraserver 
  ntcore::ntcore 
  cscore::cscore 
  opencv 
  wpinet::wpinet 
  wpiutil::wpiutil
)

if(TOOLCHAIN_TRIPLE STREQUAL "arm-nilrt-linux-gnueabi")
  target_link_libraries(test_exec nilibraries dl)
endif()

if(WIN32)
  add_custom_command(TARGET test_exec POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:test_exec> $<TARGET_FILE_DIR:test_exec>
    COMMAND_EXPAND_LISTS
  )
endif()