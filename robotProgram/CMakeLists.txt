add_executable(robotProgram)

set_property(TARGET robotProgram PROPERTY CXX_STANDARD 20)
set_property(TARGET robotProgram PROPERTY CXX_STANDARD_REQUIRED ON)

if(MSVC)
    target_compile_options(robotProgram PRIVATE /Zc:preprocessor /bigobj)
endif()

target_include_directories(robotProgram PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include" ${OPENGL_INCLUDE_DIRS})

add_subdirectory(src)

target_link_libraries(
    robotProgram
    ${CMAKE_DL_LIBS}
    fieldImages::fieldImages
    wpigui::wpigui
    Threads::Threads
    wpilibnewcommands
    libssh
    apriltag::apriltag
    googletest::googletest
)

if(WIN32)
    add_custom_command(
        TARGET robotProgram
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:robotProgram>
            $<TARGET_FILE_DIR:robotProgram>
        COMMAND_EXPAND_LISTS
    )
endif()
