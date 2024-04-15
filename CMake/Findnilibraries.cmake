include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

list(APPEND CMAKE_FIND_LIBRARY_SUFFIXES .so.24.0.0)
list(APPEND CMAKE_FIND_LIBRARY_SUFFIXES .so.23.3.0)

list(APPEND runtime_libs "embcanshim")
list(APPEND runtime_libs "fpgalvshim")

FindWpiPackage(chipobject "libRoboRIO_FRC_ChipObject.so.24.0.0" "2024.2.1" "" YES NO GetNiUrl)
FindWpiPackage(netcomm "libFRC_NetworkCommunication.so.24.0.0" "2024.2.1" "" YES NO GetNiUrl)
FindWpiPackage(runtime "${runtime_libs}" "2024.2.1" "" YES YES GetNiUrl)
FindWpiPackage(visa "" "2024.2.1" "" YES NO GetNiUrl)

add_library(nilibraries INTERFACE)
target_link_libraries(nilibraries INTERFACE chipobject netcomm runtime visa)