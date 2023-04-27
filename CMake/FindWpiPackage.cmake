macro(FindWpiPackage packageName version)

include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

GetWpiUrl(${packageName} ${version})

FetchContent_Declare(
  ${packageName}_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(${packageName}_headers)

FetchContent_Declare(
  ${packageName}_libs
  URL ${LIB_URL}
)

FetchContent_MakeAvailable(${packageName}_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

if(WIN32)
  find_file(${packageName}_DLL
    NAMES ${packageName}${DEBUG_STRING}.dll
    HINTS ${${packageName}_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endif()

find_library(${packageName}_LIBRARY 
  NAMES ${packageName}${DEBUG_STRING}
  HINTS ${${packageName}_libs_SOURCE_DIR} 
  PATH_SUFFIXES ${PATH_SUFFIX}
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
  endif()
endif()

endmacro(FindWpiPackage)
