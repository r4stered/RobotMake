include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getphotonvisionurl("v2023.4.2")

FetchContent_Declare(photonvision_headers URL ${HEADER_URL})

FetchContent_Declare(photonvision_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(photonvision_headers)

FetchContent_Declare(photonvision_libs URL ${LIB_URL})
FetchContent_MakeAvailable(photonvision_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

cmake_print_variables(photonvision_headers_SOURCE_DIR)
cmake_print_variables(PATH_SUFFIX)

if(WIN32)
  find_file(
    PHOTONVISION_DLL
    NAMES "Photon.dll"
    HINTS ${photonvision_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
endif()

find_library(
  PHOTONVISION_LIBRARY
  NAMES "Photon"
  HINTS ${photonvision_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(PHOTONVISION_HEADERS ${photonvision_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(photonvision DEFAULT_MSG PHOTONVISION_HEADERS
                                  PHOTONVISION_LIBRARY)

mark_as_advanced(PHOTONVISION_HEADERS PHOTONVISION_LIBRARY)

if(PHOTONVISION_FOUND AND NOT TARGET photonvision::photonvision)
  if(GET_SHARED_LIBS)
    add_library(photonvision::photonvision SHARED IMPORTED)
  else()
    add_library(photonvision::photonvision STATIC IMPORTED)
  endif()

  if(WIN32)
    set_target_properties(
      photonvision::photonvision
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PHOTONVISION_HEADERS}
                 IMPORTED_LOCATION ${PHOTONVISION_DLL}
                 IMPORTED_IMPLIB ${PHOTONVISION_LIBRARY})
  else()
    set_target_properties(
      photonvision::photonvision
      PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${PHOTONVISION_HEADERS}
                 IMPORTED_LOCATION ${PHOTONVISION_LIBRARY})

    putlibsindeployfolder(${PHOTONVISION_LIBRARY})
  endif()
endif()
