include(CMakePrintHelpers)

function(GetWpiUrl library_name version)
    set(OS_STRING ${CMAKE_SYSTEM_NAME})
    string(TOLOWER ${OS_STRING} OS_STRING)

    if(NOT DEFINED TOOLCHAIN_TRIPLE)
        if(OS_STRING STREQUAL "Darwin")
            set(OS_STRING "macos")
            set(ARCH_STRING "universal")
        else()
            set(ARCH_STRING "x86-64")
        endif()
    else()
        if(${TOOLCHAIN_TRIPLE} STREQUAL "arm-nilrt-linux-gnueabi")
            set(ARCH_STRING "athena")
        endif()
    endif()

    if(GET_SHARED_LIBS)
        set(STATIC_STRING "")
    else()
        set(STATIC_STRING "static")
    endif()

    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/${library_name}/${library_name}-cpp/${version}/${library_name}-cpp-${version}-")

    set(HEADER_URL "${BASE_URL}headers.zip" PARENT_SCOPE)
    set(LIB_URL "${BASE_URL}${OS_STRING}${ARCH_STRING}${STATIC_STRING}.zip" PARENT_SCOPE)

    if(GET_SHARED_LIBS)
        set(STATIC_STRING "shared")
    endif()
    cmake_print_variables(STATIC_STRING)

    set(PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${STATIC_STRING}" PARENT_SCOPE)
endfunction()