include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

GetWpiUrl(wpiutil 2023.4.3)

FetchContent_Declare(
  wpiutil_headers
  URL  ${HEADER_URL}
)
FetchContent_MakeAvailable(wpiutil_headers)

FetchContent_Declare(
  wpiutil_libs
  URL  ${LIB_URL}
)

FetchContent_MakeAvailable(wpiutil_libs)

cmake_print_variables(PATH_SUFFIX)

if(WIN32)
  find_file(WPIUTIL_DLL
    NAMES wpiutil.dll
    HINTS ${wpiutil_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endif()

find_library(WPIUTIL_LIBRARY 
  NAMES wpiutil 
  HINTS ${wpiutil_libs_SOURCE_DIR} 
  PATH_SUFFIXES ${PATH_SUFFIX}
  REQUIRED
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)

set(WPIUTIL_HEADERS ${wpiutil_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  wpiutil 
  DEFAULT_MSG
  WPIUTIL_LIBRARY
  WPIUTIL_HEADERS
)

mark_as_advanced(WPIUTIL_LIBRARY WPIUTIL_HEADERS)

if(WPIUTIL_FOUND AND NOT TARGET wpiutil::wpiutil)
  if(GET_SHARED_LIBS)
    add_library(wpiutil::wpiutil SHARED IMPORTED)
  else()
    add_library(wpiutil::wpiutil STATIC IMPORTED)
  endif()

  if(WIN32)
    set_target_properties(
      wpiutil::wpiutil
      PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${WPIUTIL_HEADERS}"
        IMPORTED_LOCATION ${WPIUTIL_DLL}
        IMPORTED_IMPLIB ${WPIUTIL_LIBRARY}
    )
  else()
    set_target_properties(
      wpiutil::wpiutil
      PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${WPIUTIL_HEADERS}"
        IMPORTED_LOCATION ${WPIUTIL_LIBRARY}
    )
  endif()
endif()
