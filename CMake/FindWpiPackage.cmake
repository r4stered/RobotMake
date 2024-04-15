function(FindWpiPackage packageName lib_names version install_folder has_debug_postfix url_func)
    include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/DeployUtils.cmake)
    include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

    # This is janky but it gets rid of a lot of duplicate code
    cmake_language(EVAL CODE "${url_func}(${packageName} ${version})")

    # Makes sure frc libs are under an frc folder to keep compatibility with gradlerio programs
    if(NOT "${install_folder}" STREQUAL "")
        set(INSTALL_FOLDER_STR ${install_folder})
    else()
        set(INSTALL_FOLDER_STR "")
    endif()

    # Handle the case where the package name is different than the library file name
    if("${lib_names}" STREQUAL "")
        set(lib_names ${packageName})
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

    if(${has_debug_postfix})
        if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
            set(DEBUG_STRING "d")
        else()
            set(DEBUG_STRING "")
        endif()
    endif()

    foreach(lib_name ${lib_names})
        if(WIN32)
            find_file(
                ${lib_name}_DLL
                NAMES ${LIB_PREFIX}${lib_name}${DEBUG_STRING}.dll
                HINTS ${${packageName}_libs_SOURCE_DIR}
                PATH_SUFFIXES ${${packageName}_PATH_SUFFIX}
                REQUIRED
                NO_DEFAULT_PATH
                NO_CMAKE_FIND_ROOT_PATH
            )
        endif()

        find_library(
            ${lib_name}_LIBRARY
            NAMES ${LIB_PREFIX}${lib_name}${DEBUG_STRING}
            HINTS ${${packageName}_libs_SOURCE_DIR}
            PATH_SUFFIXES ${${packageName}_PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )
    endforeach()

    set(${packageName}_HEADERS ${${packageName}_headers_SOURCE_DIR})

    include(FindPackageHandleStandardArgs)

    foreach(lib_name ${lib_names})
        list(APPEND all_lib_files ${lib_name}_LIBRARY)
        list(APPEND all_lib_files ${packageName}_HEADERS)
    endforeach()
    

    find_package_handle_standard_args(
        ${packageName}
        DEFAULT_MSG
        ${all_lib_files}
    )

    mark_as_advanced(${all_lib_files})

    add_library(${packageName} INTERFACE)

    if(${packageName}_FOUND AND NOT TARGET ${packageName}::${packageName})
        foreach(lib_name ${lib_names})        
            if(GET_SHARED_LIBS)
                add_library(${lib_name}::${lib_name} SHARED IMPORTED)
            else()
                add_library(${lib_name}::${lib_name} STATIC IMPORTED)
            endif()

            if(WIN32)
                set_target_properties(
                    ${lib_name}::${lib_name}
                    PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                        IMPORTED_LOCATION ${${lib_name}_DLL}
                        IMPORTED_IMPLIB ${${lib_name}_LIBRARY}
                )
            else()
                set_target_properties(
                    ${lib_name}::${lib_name}
                    PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                        IMPORTED_LOCATION ${${lib_name}_LIBRARY}
                )

                putlibsindeployfolder(${${lib_name}_LIBRARY})
            endif()
            message("LIB NAME IS: ${lib_name}")
            target_link_libraries(${packageName} INTERFACE ${lib_name}::${lib_name})
        endforeach()
    endif()
endfunction(FindWpiPackage)
