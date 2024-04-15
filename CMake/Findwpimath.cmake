include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpimath "" "" "" YES NO GetWpiUrl)

target_link_libraries(wpimath::wpimath INTERFACE wpiutil::wpiutil)
