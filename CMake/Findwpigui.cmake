include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

FindWpiPackage(wpigui "" "" "" NO GetWpiUrl)

target_link_libraries(wpigui::wpigui INTERFACE imgui)