include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getphotonvisionurl("photonlib" "v2024.3.1")

fetchcontent_declare(photonlib_headers URL ${HEADER_URL})
fetchcontent_makeavailable(photonlib_headers)

fetchcontent_declare(photonlib_libs URL ${LIB_URL})
fetchcontent_makeavailable(photonlib_libs)

getphotonvisionurl("photontargeting" "v2024.3.1")

fetchcontent_declare(photontargeting_headers URL ${HEADER_URL})
fetchcontent_makeavailable(photontargeting_headers)

fetchcontent_declare(photontargeting_libs URL ${LIB_URL})
fetchcontent_makeavailable(photontargeting_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(DEBUG_STRING "d")
else()
    set(DEBUG_STRING "")
endif()

if(WIN32)
    find_file(
        PHOTONLIB_DLL
        NAMES "photonlib.dll"
        HINTS ${photonlib_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(
        PHOTONTARGETING_DLL
        NAMES "photontargeting.dll"
        HINTS ${photontargeting_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
endif()

find_library(
    PHOTONLIB_LIBRARY
    NAMES "photonlib"
    HINTS ${photonlib_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    PHOTONTARGETING_LIBRARY
    NAMES "photontargeting"
    HINTS ${photontargeting_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

set(PHOTONLIB_HEADERS ${photonlib_headers_SOURCE_DIR})
set(PHOTONTARGETING_HEADERS ${photontargeting_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    photonvision
    DEFAULT_MSG
    PHOTONLIB_HEADERS
    PHOTONLIB_LIBRARY
    PHOTONTARGETING_HEADERS
    PHOTONTARGETING_LIBRARY
)

mark_as_advanced(
    PHOTONLIB_HEADERS
    PHOTONLIB_LIBRARY
    PHOTONTARGETING_HEADERS
    PHOTONTARGETING_LIBRARY
)

add_library(photonvision INTERFACE)

if(PHOTONVISION_FOUND AND NOT TARGET photonvision::photonvision)
    if(GET_SHARED_LIBS)
        add_library(photonlib::photonlib SHARED IMPORTED)
        add_library(photontargeting::photontargeting SHARED IMPORTED)
    else()
        add_library(photonlib::photonlib STATIC IMPORTED)
        add_library(photontargeting::photontargeting STATIC IMPORTED)
    endif()

    if(WIN32)
        set_target_properties(
            photonlib::photonlib
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PHOTONLIB_HEADERS}
                IMPORTED_LOCATION ${PHOTONLIB_DLL}
                IMPORTED_IMPLIB ${PHOTONLIB_LIBRARY}
        )
        set_target_properties(
            photontargeting::photontargeting
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PHOTONTARGETING_HEADERS}
                IMPORTED_LOCATION ${PHOTONTARGETING_DLL}
                IMPORTED_IMPLIB ${PHOTONTARGETING_LIBRARY}
        )
    else()
        set_target_properties(
            photonlib::photonlib
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PHOTONLIB_HEADERS}
                IMPORTED_LOCATION ${PHOTONLIB_LIBRARY}
        )

        set_target_properties(
            photontargeting::photontargeting
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${PHOTONTARGETING_HEADERS}
                IMPORTED_LOCATION ${PHOTONTARGETING_LIBRARY}
        )

        putlibsindeployfolder(${PHOTONLIB_LIBRARY})
        putlibsindeployfolder(${PHOTONTARGETING_LIBRARY})
    endif()
    target_link_libraries(
        photonvision
        INTERFACE photonlib::photonlib photontargeting::photontargeting
    )
endif()
