include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getplayingwithfusionurl("driver" "2024.03.09")

fetchcontent_declare(playingwithfusiondriver_headers URL ${HEADER_URL})
fetchcontent_makeavailable(playingwithfusiondriver_headers)

fetchcontent_declare(playingwithfusiondriver_libs URL ${LIB_URL})
fetchcontent_makeavailable(playingwithfusiondriver_libs)

getplayingwithfusionurl("cpp" "2024.03.09")

fetchcontent_declare(playingwithfusioncpp_headers URL ${HEADER_URL})
fetchcontent_makeavailable(playingwithfusioncpp_headers)

fetchcontent_declare(playingwithfusioncpp_libs URL ${LIB_URL})
fetchcontent_makeavailable(playingwithfusioncpp_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(DEBUG_STRING "d")
else()
    set(DEBUG_STRING "")
endif()

if(WIN32)
    find_file(
        PLAYINGWITHFUSIONDRIVER_DLL
        NAMES "PlayingWithFusionDriver.dll"
        HINTS ${playingwithfusiondriver_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(
        PLAYINGWITHFUSIONCPP_DLL
        NAMES "PlayingWithFusion.dll"
        HINTS ${playingwithfusioncpp_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
endif()

find_library(
    PLAYINGWITHFUSIONDRIVER_LIBRARY
    NAMES "PlayingWithFusionDriver"
    HINTS ${playingwithfusiondriver_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    PLAYINGWITHFUSIONCPP_LIBRARY
    NAMES "PlayingWithFusion"
    HINTS ${playingwithfusioncpp_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

set(PLAYINGWITHFUSIONDRIVER_HEADERS ${playingwithfusiondriver_headers_SOURCE_DIR})
set(PLAYINGWITHFUSIONCPP_HEADERS ${playingwithfusioncpp_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    playingwithfusion
    DEFAULT_MSG
    PLAYINGWITHFUSIONDRIVER_HEADERS
    PLAYINGWITHFUSIONDRIVER_LIBRARY
    PLAYINGWITHFUSIONCPP_HEADERS
    PLAYINGWITHFUSIONCPP_LIBRARY
)

mark_as_advanced(
    PLAYINGWITHFUSIONDRIVER_HEADERS
    PLAYINGWITHFUSIONDRIVER_LIBRARY
    PLAYINGWITHFUSIONCPP_HEADERS
    PLAYINGWITHFUSIONCPP_LIBRARY
)

add_library(playingwithfusion INTERFACE)

if(PLAYINGWITHFUSION_FOUND AND NOT TARGET playingwithfusion::playingwithfusion)
    if(GET_SHARED_LIBS)
        add_library(playingwithfusiondriver::playingwithfusiondriver SHARED IMPORTED)
        add_library(playingwithfusioncpp::playingwithfusioncpp SHARED IMPORTED)
    else()
        add_library(playingwithfusiondriver::playingwithfusiondriver STATIC IMPORTED)
        add_library(playingwithfusioncpp::playingwithfusioncpp SHARED IMPORTED)
    endif()

    if(WIN32)
        set_target_properties(
            playingwithfusiondriver::playingwithfusiondriver
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PLAYINGWITHFUSIONDRIVER_HEADERS}
                IMPORTED_LOCATION ${PLAYINGWITHFUSIONDRIVER_DLL}
                IMPORTED_IMPLIB ${PLAYINGWITHFUSIONDRIVER_LIBRARY}
        )
        set_target_properties(
            playingwithfusioncpp::playingwithfusioncpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PLAYINGWITHFUSIONCPP_HEADERS}
                IMPORTED_LOCATION ${PLAYINGWITHFUSIONCPP_DLL}
                IMPORTED_IMPLIB ${PLAYINGWITHFUSIONCPP_LIBRARY}
        )
    else()
        set_target_properties(
            playingwithfusiondriver::playingwithfusiondriver
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PLAYINGWITHFUSIONDRIVER_HEADERS}
                IMPORTED_LOCATION ${PLAYINGWITHFUSIONDRIVER_LIBRARY}
        )
        set_target_properties(
            playingwithfusioncpp::playingwithfusioncpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PLAYINGWITHFUSIONCPP_HEADERS}
                IMPORTED_LOCATION ${PLAYINGWITHFUSIONCPP_LIBRARY}
        )

        putlibsindeployfolder(${PLAYINGWITHFUSIONDRIVER_LIBRARY})
        putlibsindeployfolder(${PLAYINGWITHFUSIONCPP_LIBRARY})
    endif()
    target_link_libraries(
        playingwithfusion
        INTERFACE
            playingwithfusioncpp::playingwithfusioncpp
            playingwithfusiondriver::playingwithfusiondriver
    )
endif()
