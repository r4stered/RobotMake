include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getchoreourl("2024.2.2")

set(INSTALL_FOLDER_STR "choreo")

FetchContent_Declare(
  choreo_headers 
  URL ${HEADER_URL}
  SOURCE_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/choreo_headers-src/${INSTALL_FOLDER_STR}
  BINARY_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/choreo_headers-build/${INSTALL_FOLDER_STR}
  SUBBUILD_DIR
    ${CMAKE_CURRENT_BINARY_DIR}/_deps/choreo_headers-subbuild/${INSTALL_FOLDER_STR}
)

FetchContent_MakeAvailable(choreo_headers)

cmake_path(GET choreo_headers_SOURCE_DIR PARENT_PATH
           choreo_FIXED_PATH)
set(choreo_headers_SOURCE_DIR ${choreo_FIXED_PATH})

FetchContent_Declare(choreo_libs URL ${LIB_URL})
FetchContent_MakeAvailable(choreo_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

if(WIN32)
  find_file(
    CHOREO_DLL
    NAMES "ChoreoLib.dll"
    HINTS ${choreo_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
endif()

find_library(
  CHOREO_LIBRARY
  NAMES "ChoreoLib"
  HINTS ${choreo_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(CHOREO_HEADERS ${choreo_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(choreo DEFAULT_MSG CHOREO_HEADERS
                                  CHOREO_LIBRARY)

mark_as_advanced(CHOREO_HEADERS CHOREO_LIBRARY)

if(CHOREO_FOUND AND NOT TARGET choreo::choreo)
  if(GET_SHARED_LIBS)
    add_library(choreo::choreo SHARED IMPORTED)
  else()
    add_library(choreo::choreo STATIC IMPORTED)
  endif()

  if(WIN32)
    set_target_properties(
        choreo::choreo
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${CHOREO_HEADERS}
                 IMPORTED_LOCATION ${CHOREO_DLL}
                 IMPORTED_IMPLIB ${CHOREO_LIBRARY})
  else()
    set_target_properties(
      choreo::choreo
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${CHOREO_HEADERS}
                 IMPORTED_LOCATION ${CHOREO_LIBRARY})

    putlibsindeployfolder(${CHOREO_LIBRARY})
  endif()
endif()
