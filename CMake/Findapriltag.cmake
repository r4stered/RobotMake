include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(apriltag "" "" "frc" YES GetWpiUrl)

target_link_libraries(apriltag::apriltag INTERFACE wpimath::wpimath apriltaglib::apriltaglib)