include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(ntcore "" "" "" YES NO GetWpiUrl)

target_link_libraries(ntcore::ntcore INTERFACE wpinet::wpinet)