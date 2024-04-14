cmake_minimum_required(VERSION 3.25)

include(CMakePrintHelpers)
include(FetchContent)

if(CMAKE_HOST_UNIX)
    fetchcontent_declare(
        rio_toolchain
        URL
            https://github.com/wpilibsuite/opensdk/releases/download/v2024-1/cortexa9_vfpv3-roborio-academic-2024-x86_64-linux-gnu-Toolchain-12.1.0.tgz
    )
elseif(CMAKE_HOST_WIN32)
    fetchcontent_declare(
        rio_toolchain
        URL
            https://github.com/wpilibsuite/opensdk/releases/download/v2024-1/cortexa9_vfpv3-roborio-academic-2024-x86_64-w64-mingw32-Toolchain-12.1.0.zip
    )
endif()
fetchcontent_makeavailable(rio_toolchain)

project(rio_compile_ex)

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
find_package(choreo REQUIRED)
find_package(playingwithfusion REQUIRED)
# if we are not building for orange pi
if(NOT TOOLCHAIN_TRIPLE STREQUAL "aarch64-none-linux-gnu-")
    # find_package(kauailabs REQUIRED)
    find_package(pathplanner REQUIRED)
    find_package(photonvision REQUIRED)
    find_package(rev REQUIRED)
    find_package(phoenix REQUIRED)
endif()
find_package(Threads REQUIRED)

if(TOOLCHAIN_TRIPLE STREQUAL "arm-nilrt-linux-gnueabi")
    find_package(nilibraries REQUIRED)
else()
    find_package(halsim_gui REQUIRED)
endif()

# if we are not building for orange pi
if(NOT TOOLCHAIN_TRIPLE STREQUAL "aarch64-none-linux-gnu-")
    add_subdirectory(robotProgram)
endif()

#add_subdirectory(vision)
