include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getkauailabsurl("2024.1.0")

FetchContent_Declare(kauailabs_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(kauailabs_headers)

FetchContent_Declare(kauailabs_libs URL ${LIB_URL})
FetchContent_MakeAvailable(kauailabs_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

if(WIN32)
  find_file(
    KAUAILABS_DLL
    NAMES "NavX.dll"
    HINTS ${kauailabs_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
endif()

find_library(
  KAUAILABS_LIBRARY
  NAMES "NavX"
  HINTS ${kauailabs_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(KAUAILABS_HEADERS ${kauailabs_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(kauailabs DEFAULT_MSG KAUAILABS_HEADERS
                                  KAUAILABS_LIBRARY)

mark_as_advanced(KAUAILABS_HEADERS KAUAILABS_LIBRARY)

if(KAUAILABS_FOUND AND NOT TARGET kauailabs::kauailabs)
  if(GET_SHARED_LIBS)
    add_library(kauailabs::kauailabs SHARED IMPORTED)
  else()
    add_library(kauailabs::kauailabs STATIC IMPORTED)
  endif()

  if(WIN32)
    set_target_properties(
      kauailabs::kauailabs
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${KAUAILABS_HEADERS}
                 IMPORTED_LOCATION ${KAUAILABS_DLL}
                 IMPORTED_IMPLIB ${KAUAILABS_LIBRARY})
  else()
    set_target_properties(
      kauailabs::kauailabs
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${KAUAILABS_HEADERS}
                 IMPORTED_LOCATION ${KAUAILABS_LIBRARY})

    putlibsindeployfolder(${KAUAILABS_LIBRARY})
  endif()
endif()
