include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpinet "" "" "" YES GetWpiUrl)

target_link_libraries(wpinet::wpinet INTERFACE wpiutil::wpiutil)