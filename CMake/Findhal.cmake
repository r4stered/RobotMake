include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(hal "wpiHal" "" "hal" YES NO GetWpiUrl)

target_link_libraries(hal INTERFACE wpiutil::wpiutil)