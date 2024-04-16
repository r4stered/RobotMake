include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

set(CTRE_VERSION "24.2.0")

FindWpiPackage(tools "CTRE_PhoenixTools" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(wpiapi-cpp "CTRE_Phoenix6_WPI" "${CTRE_VERSION}" "" NO NO GetCtreUrl)

add_library(phoenix6 INTERFACE)
target_link_libraries(phoenix6 INTERFACE tools wpiapi-cpp)