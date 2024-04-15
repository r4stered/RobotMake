include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpimath "")

target_link_libraries(wpimath::wpimath INTERFACE wpiutil::wpiutil)
