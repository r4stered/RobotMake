function(FindWpiPackage packageName version install_folder url_func)
    include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/DeployUtils.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

    # This is janky but it gets rid of a lot of duplicate code
    cmake_language(EVAL CODE "${url_func}(${packageName} ${version})")

    # TODO: not sure if needed
    if(NOT "${install_folder}" STREQUAL "")
        set(INSTALL_FOLDER_STR ${install_folder})
    else()
        set(INSTALL_FOLDER_STR "")
    endif()

    fetchcontent_declare(
        ${packageName}_headers
        DOWNLOAD_COMMAND
        URL
            ${${packageName}_HEADER_URL}
            SOURCE_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-src/${INSTALL_FOLDER_STR}
            BINARY_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-build/${INSTALL_FOLDER_STR}
            SUBBUILD_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-subbuild/${INSTALL_FOLDER_STR}
    )

    fetchcontent_makeavailable(${packageName}_headers)

    if(NOT "${install_folder}" STREQUAL "")
        cmake_path(GET ${packageName}_headers_SOURCE_DIR PARENT_PATH ${packageName}_FIXED_PATH)
        set(${packageName}_headers_SOURCE_DIR ${${packageName}_FIXED_PATH})
    endif()

    fetchcontent_declare(${packageName}_libs URL ${${packageName}_LIB_URL})
    fetchcontent_makeavailable(${packageName}_libs)

    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(DEBUG_STRING "d")
    else()
        set(DEBUG_STRING "")
    endif()

    if(WIN32)
        find_file(
            ${packageName}_DLL
            NAMES ${LIB_PREFIX}${packageName}${DEBUG_STRING}.dll
            HINTS ${${packageName}_libs_SOURCE_DIR}
            PATH_SUFFIXES ${${packageName}_PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )
    endif()

    find_library(
        ${packageName}_LIBRARY
        NAMES ${LIB_PREFIX}${packageName}${DEBUG_STRING}
        HINTS ${${packageName}_libs_SOURCE_DIR}
        PATH_SUFFIXES ${${packageName}_PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    set(${packageName}_HEADERS ${${packageName}_headers_SOURCE_DIR})

    include(FindPackageHandleStandardArgs)

    find_package_handle_standard_args(
        ${packageName}
        DEFAULT_MSG
        ${packageName}_LIBRARY
        ${packageName}_HEADERS
    )

    mark_as_advanced(${packageName}_LIBRARY ${packageName}_HEADERS)

    if(${packageName}_FOUND AND NOT TARGET ${packageName}::${packageName})
        if(GET_SHARED_LIBS)
            add_library(${packageName}::${packageName} SHARED IMPORTED)
        else()
            add_library(${packageName}::${packageName} STATIC IMPORTED)
        endif()

        if(WIN32)
            set_target_properties(
                ${packageName}::${packageName}
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                    IMPORTED_LOCATION ${${packageName}_DLL}
                    IMPORTED_IMPLIB ${${packageName}_LIBRARY}
            )
        else()
            set_target_properties(
                ${packageName}::${packageName}
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                    IMPORTED_LOCATION ${${packageName}_LIBRARY}
            )

            putlibsindeployfolder(${${packageName}_LIBRARY})
        endif()
    endif()
endfunction(FindWpiPackage)
