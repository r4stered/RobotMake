include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getthirdpartylibraryurl("apriltaglib" "3.3.0-1")

FetchContent_Declare(apriltaglib_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(apriltaglib_headers)

FetchContent_Declare(apriltaglib_libs URL ${LIB_URL})

FetchContent_MakeAvailable(apriltaglib_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

find_library(
  APRILTAGLIB_LIBRARY
  NAMES apriltaglib${DEBUG_STRING}
  HINTS ${apriltaglib_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(APRILTAGLIB_HEADERS ${apriltaglib_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(apriltaglib DEFAULT_MSG APRILTAGLIB_LIBRARY
                                  APRILTAGLIB_HEADERS)

mark_as_advanced(APRILTAGLIB_LIBRARY APRILTAGLIB_HEADERS)

if(apriltaglib_FOUND AND NOT TARGET apriltaglib::apriltaglib)
  add_library(apriltaglib::apriltaglib STATIC IMPORTED)

  set_target_properties(
    apriltaglib::apriltaglib
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${APRILTAGLIB_HEADERS}"
               IMPORTED_LOCATION ${APRILTAGLIB_LIBRARY})

  putlibsindeployfolder(${APRILTAGLIB_LIBRARY})
endif()
