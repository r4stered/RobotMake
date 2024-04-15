include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpilibc "" "" "" YES NO GetWpiUrl)

target_link_libraries(wpilibc::wpilibc INTERFACE hal cameraserver::cameraserver)