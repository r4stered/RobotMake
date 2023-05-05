include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

GetRevUrl("driver" "2023.1.3")

FetchContent_Declare(
  revdriver_headers
  URL ${HEADER_URL}
)

FetchContent_Declare(
  revdriver_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(revdriver_headers)

FetchContent_Declare(
  revdriver_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(revdriver_libs)

GetRevUrl("cpp" "2023.1.3")

FetchContent_Declare(
  revcpp_headers
  URL ${HEADER_URL}
)

FetchContent_Declare(
  revcpp_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(revcpp_headers)

FetchContent_Declare(
  revcpp_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(revcpp_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

cmake_print_variables(revdriver_headers_SOURCE_DIR)
cmake_print_variables(PATH_SUFFIX)

if(WIN32)
  find_file(REVDRIVER_DLL
      NAMES "REVLibDriver.dll"
      HINTS ${revdriver_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
  )

  find_file(REVCPP_DLL
    NAMES "REVLib.dll"
    HINTS ${revcpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endif()

find_library(
    REVDRIVER_LIBRARY
    NAMES "REVLibDriver"
    HINTS ${revdriver_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    REVCPP_LIBRARY
    NAMES "REVLib"
    HINTS ${revcpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

set(REVDRIVER_HEADERS ${revdriver_headers_SOURCE_DIR})
set(REVCPP_HEADERS ${revcpp_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  rev 
  DEFAULT_MSG
  REVDRIVER_HEADERS
  REVDRIVER_LIBRARY
  REVCPP_HEADERS
  REVCPP_LIBRARY
)

mark_as_advanced(REVDRIVER_HEADERS REVDRIVER_LIBRARY REVCPP_HEADERS REVCPP_LIBRARY)

add_library(rev INTERFACE)

if(REV_FOUND AND NOT TARGET rev::rev)
    if(GET_SHARED_LIBS)
        add_library(revdriver::revdriver SHARED IMPORTED)
        add_library(revcpp::revcpp SHARED IMPORTED)
    else()
        add_library(revdriver::revdriver STATIC IMPORTED)
        add_library(revcpp::revcpp SHARED IMPORTED)
    endif()

    if(WIN32)
        set_target_properties(
            revdriver::revdriver
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${REVDRIVER_HEADERS}
                IMPORTED_LOCATION ${REVDRIVER_DLL}
                IMPORTED_IMPLIB ${REVDRIVER_LIBRARY}
        )
        set_target_properties(
            revcpp::revcpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${REVCPP_HEADERS}
                IMPORTED_LOCATION ${REVCPP_DLL}
                IMPORTED_IMPLIB ${REVCPP_LIBRARY}
        )
    else()
      set_target_properties(
        revdriver::revdriver
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${REVDRIVER_HEADERS}
          IMPORTED_LOCATION ${REVDRIVER_LIBRARY}
        )
      set_target_properties(
        revcpp::revcpp
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${REVCPP_HEADERS}
          IMPORTED_LOCATION ${REVCPP_LIBRARY}
        )
    endif()
    target_link_libraries(rev INTERFACE revcpp::revcpp revdriver::revdriver)
endif()