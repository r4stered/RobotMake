include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(revlib_driver "REVLibDriver" "2024.2.4" "" NO NO GetRevUrl)
FindWpiPackage(revlib_cpp "REVLib" "2024.2.4" "" NO NO GetRevUrl)

add_library(revlib INTERFACE)
target_link_libraries(revlib INTERFACE revlib_driver revlib_cpp)