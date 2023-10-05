add_executable(visionProgram main.cpp 
  aruco/cameraparameters.cpp 
  aruco/dictionary.cpp 
  aruco/marker.cpp 
  aruco/markerdetector_impl.cpp 
  aruco/markerdetector.cpp 
  aruco/markermap.cpp 
  aruco/calibrator.cpp
  aruco/ippe.cpp
  aruco/markerlabeler.cpp
  aruco/dictionary_based.cpp
  aruco/debug.cpp
)
if(MSVC)
target_compile_options(visionProgram PRIVATE /Zc:preprocessor /bigobj)
endif()

target_link_libraries(visionProgram
  Threads::Threads 
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