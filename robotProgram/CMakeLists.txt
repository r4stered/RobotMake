add_executable(robotProgram)

target_include_directories(robotProgram PUBLIC "./include")

add_subdirectory(cpp)

if(MSVC)
target_compile_options(robotProgram PRIVATE /Zc:preprocessor /bigobj)
endif()

target_link_libraries(
robotProgram
Threads::Threads
# phoenix
# rev
# photonvision::photonvision
# pathplanner::pathplanner
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
wpiutil::wpiutil)

if(TOOLCHAIN_TRIPLE STREQUAL "arm-nilrt-linux-gnueabi")
target_link_libraries(robotProgram nilibraries dl)
endif()

if(WIN32)
add_custom_command(
  TARGET robotProgram
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:robotProgram>
          $<TARGET_FILE_DIR:robotProgram>
  COMMAND_EXPAND_LISTS)
endif()