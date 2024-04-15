include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpilibnewcommands "wpilibNewCommands" "" "frc2" YES GetWpiUrl)

target_link_libraries(wpilibnewcommands INTERFACE wpilibc::wpilibc)