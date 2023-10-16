file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${rio_compile_ex_SOURCE_DIR}/robotProgram/str/*.h")

target_sources(robotProgram PUBLIC ${HEADER_LIST})