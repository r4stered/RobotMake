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
find_package(wpimath REQUIRED)
find_package(Threads REQUIRED)
find_package(googletest REQUIRED)
find_package(apriltaglib REQUIRED)
find_package(libssh REQUIRED)
find_package(opencv REQUIRED)
find_package(apriltag REQUIRED)
find_package(cscore REQUIRED)
find_package(ntcore REQUIRED)
find_package(cameraserver REQUIRED)
find_package(hal)
find_package(wpilibc REQUIRED)
find_package(wpilibnewcommands REQUIRED)
#find_package(imgui REQUIRED)
#find_package(wpigui REQUIRED)
#find_package(fieldimages REQUIRED)
find_package(ntcoreffi REQUIRED)
find_package(nilibraries REQUIRED)

add_subdirectory(robotProgram)
