include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(cscore "" "" "" YES NO GetWpiUrl)

target_link_libraries(cscore::cscore INTERFACE opencv wpinet::wpinet)