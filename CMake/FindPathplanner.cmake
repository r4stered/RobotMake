include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getpathplannerurl("2024.0.0-beta-1")

set(INSTALL_FOLDER_STR "pathplanner")

FetchContent_Declare(pathplanner_headers URL ${HEADER_URL})

FetchContent_Populate(
  pathplanner_headers
  DOWNLOAD_COMMAND
  URL ${HEADER_URL}
  SOURCE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/pathplanner_headers-src/${INSTALL_FOLDER_STR}
  BINARY_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/pathplanner_headers-build/${INSTALL_FOLDER_STR}
  SUBBUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/pathplanner_headers-subbuild/${INSTALL_FOLDER_STR}
)

cmake_path(GET pathplanner_headers_SOURCE_DIR PARENT_PATH
           pathplanner_FIXED_PATH)
set(pathplanner_headers_SOURCE_DIR ${pathplanner_FIXED_PATH})

FetchContent_Declare(pathplanner_libs URL ${LIB_URL})
FetchContent_MakeAvailable(pathplanner_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

cmake_print_variables(pathplanner_headers_SOURCE_DIR)
cmake_print_variables(PATH_SUFFIX)

if(WIN32)
  find_file(
    PATHPLANNER_DLL
    NAMES "PathplannerLib.dll"
    HINTS ${pathplanner_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
endif()

find_library(
  PATHPLANNER_LIBRARY
  NAMES "PathplannerLib"
  HINTS ${pathplanner_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(PATHPLANNER_HEADERS ${pathplanner_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(pathplanner DEFAULT_MSG PATHPLANNER_HEADERS
                                  PATHPLANNER_LIBRARY)

mark_as_advanced(PATHPLANNER_HEADERS PATHPLANNER_LIBRARY)

if(PATHPLANNER_FOUND AND NOT TARGET pathplanner::pathplanner)
  if(GET_SHARED_LIBS)
    add_library(pathplanner::pathplanner SHARED IMPORTED)
  else()
    add_library(pathplanner::pathplanner STATIC IMPORTED)
  endif()

  if(WIN32)
    set_target_properties(
      pathplanner::pathplanner
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PATHPLANNER_HEADERS}
                 IMPORTED_LOCATION ${PATHPLANNER_DLL}
                 IMPORTED_IMPLIB ${PATHPLANNER_LIBRARY})
  else()
    set_target_properties(
      pathplanner::pathplanner
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PATHPLANNER_HEADERS}
                 IMPORTED_LOCATION ${PATHPLANNER_LIBRARY})

    putlibsindeployfolder(${PATHPLANNER_LIBRARY})
  endif()
endif()
