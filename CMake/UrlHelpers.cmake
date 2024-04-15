include(CMakePrintHelpers)
include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/Versions.cmake)

# This function sets OS_STRING equal to the appropriate url string for downloading wpi deps
function(GetOsNameForUrl)
    set(OS_STRING ${CMAKE_SYSTEM_NAME})
    if(OS_STRING STREQUAL "Darwin")
        set(OS_STRING "macos")
    endif()
    string(TOLOWER ${OS_STRING} OS_STRING)  

    return(PROPAGATE OS_STRING)
endfunction(GetOsNameForUrl)

# This function sets BUILD_TYPE_STRING equal to the appropriate url string for downloading wpi deps
function(GetBuildTypeForUrl)
    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(BUILD_TYPE_STRING "debug")
    else()
        set(BUILD_TYPE_STRING "")
    endif()

    return(PROPAGATE BUILD_TYPE_STRING)
endfunction(GetBuildTypeForUrl)

# This function sets ARCH_STRING equal to the appropriate url string for downloading wpi deps
function(GetArchStringForUrl)
    # If we are building locally, we won't have a toolchain file defined
    if(NOT DEFINED TOOLCHAIN_TRIPLE)
        if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
            set(ARCH_STRING "universal")
        # Asssume for now we are building from x86 if local. TODO: Handle local arm builds
        else()
            set(ARCH_STRING "x86-64")
        endif()
    else()
        # Rio build
        if(${TOOLCHAIN_TRIPLE} STREQUAL "arm-nilrt-linux-gnueabi")
            set(ARCH_STRING "athena")
        # Generic arm build
        elseif(${TOOLCHAIN_TRIPLE} STREQUAL "aarch64-none-linux-gnu-")
            set(ARCH_STRING "arm64")
        endif()
    endif()

    return(PROPAGATE ARCH_STRING)
endfunction(GetArchStringForUrl)

# This function sets SHARED_STRING equal to the appropriate url string for downloading wpi deps
# TODO: Some libraries are only avaliable as shared or static. Make sure to handle this here??
function(GetSharedOrStaticStringForUrl is_shared)
    if(${is_shared})
        set(SHARED_STRING "")
    else()
        set(SHARED_STRING "static")
    endif()

    return(PROPAGATE SHARED_STRING)
endfunction(GetSharedOrStaticStringForUrl)

# Gets the header and lib url for a third party wpi packages. All of these are only static except opencv
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetThirdpartyUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # These libs are avaliable as static only, so force it
    if(${library_name} STREQUAL "apriltaglib" OR
       ${library_name} STREQUAL "googletest" OR
       ${library_name} STREQUAL "imgui" OR
       ${library_name} STREQUAL "libssh"
    )
        set(SHARED_STRING "static")
    endif()


    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/frc${WPI_YEAR}/${library_name}/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    if(${library_name} STREQUAL "opencv")
        set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/frc${WPI_YEAR}/${library_name}/${library_name}-cpp/${version}")
        set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${version}-headers.zip")
        set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    # Libssh has the os in the subdirectories
    if("${library_name}" STREQUAL "libssh" OR
       "${library_name}" STREQUAL "opencv")
        set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")
    endif()

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetThirdpartyUrl)

# Gets the header and lib url for a wpi package. You must make sure GET_SHARED_LIBS is set to True or False before calling this
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetWpiUrl library_name)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # These libs are avaliable as static only, so force it
    if(${library_name} STREQUAL "wpigui")
        set(SHARED_STRING "static")
    endif()

    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/${library_name}/${library_name}-cpp/${WPI_VERSION}")

    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${WPI_VERSION}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${WPI_VERSION}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    #Special case for wierd capitilization of commands v2
    if(${library_name} STREQUAL "wpilibnewcommands")
        string(REPLACE "wpilibnewcommands" "wpilibNewCommands" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "wpilibnewcommands" "wpilibNewCommands" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    #Special case for wierd capitilization of field images
    if(${library_name} STREQUAL "fieldimages")
        string(REPLACE "fieldimages" "fieldImages" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "fieldimages" "fieldImages" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()
    set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetWpiUrl)
