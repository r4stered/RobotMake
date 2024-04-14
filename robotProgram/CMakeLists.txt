add_executable(robotProgram)

set_property(TARGET robotProgram PROPERTY CXX_STANDARD 20)
set_property(TARGET robotProgram PROPERTY CXX_STANDARD_REQUIRED ON)

if(MSVC)
    target_compile_options(robotProgram PRIVATE /Zc:preprocessor /bigobj)
endif()

target_include_directories(robotProgram PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")

add_subdirectory(src)

target_link_libraries(
    robotProgram
    Threads::Threads
    phoenix
    rev
    photonvision
    playingwithfusion
    choreo::choreo
    pathplanner::pathplanner
    # kauailabs::kauailabs
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
    target_link_libraries(robotProgram nilibraries dl)
endif()

if(WIN32)
    add_custom_command(
        TARGET robotProgram
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:robotProgram>
            $<TARGET_FILE_DIR:robotProgram>
        COMMAND_EXPAND_LISTS
    )
endif()
