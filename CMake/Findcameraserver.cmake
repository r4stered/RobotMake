include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(cameraserver "" "" "" YES NO GetWpiUrl)

target_link_libraries(cameraserver::cameraserver INTERFACE ntcore::ntcore cscore::cscore)