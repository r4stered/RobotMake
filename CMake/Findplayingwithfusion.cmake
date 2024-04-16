include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(playingwithfusion_driver "PlayingWithFusionDriver" "2024.03.09" "" NO NO GetPlayingWithFusionUrl)
FindWpiPackage(playingwithfusion_cpp "PlayingWithFusion" "2024.03.09" "" NO NO GetPlayingWithFusionUrl)

add_library(playingwithfusion INTERFACE)
target_link_libraries(playingwithfusion INTERFACE playingwithfusion_driver playingwithfusion_cpp)