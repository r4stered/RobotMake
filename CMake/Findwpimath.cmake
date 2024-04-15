include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpimath "" "" GetWpiUrl)

target_link_libraries(wpimath::wpimath INTERFACE wpiutil::wpiutil)
