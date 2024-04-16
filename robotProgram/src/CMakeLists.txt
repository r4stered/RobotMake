file(GLOB_RECURSE HEADER_LIST CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/robotProgram/include/*.h")

cmake_print_variables(HEADER_LIST)

target_sources(robotProgram PRIVATE RobotContainer.cpp Constants.cpp Robot.cpp ${HEADER_LIST})

add_subdirectory(subsystems)
add_subdirectory(str)

source_group(
    TREE "${PROJECT_SOURCE_DIR}/robotProgram/include"
    PREFIX "Header Files"
    FILES ${HEADER_LIST}
)
