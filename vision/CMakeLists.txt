add_executable(visionProgram main.cpp)
if(MSVC)
target_compile_options(visionProgram PRIVATE /Zc:preprocessor /bigobj)
endif()

target_link_libraries(visionProgram 
  Threads::Threads 
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
  target_link_libraries(visionProgram nilibraries dl)
endif()

if(WIN32)
  add_custom_command(TARGET visionProgram POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:visionProgram> $<TARGET_FILE_DIR:visionProgram>
    COMMAND_EXPAND_LISTS
  )
endif()