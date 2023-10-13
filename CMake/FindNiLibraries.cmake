include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

getniurl("chipobject" "2023.3.0")

FetchContent_Declare(chipobject_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(chipobject_headers)

FetchContent_Declare(chipobject_libs URL ${LIB_URL})
FetchContent_MakeAvailable(chipobject_libs)

getniurl("netcomm" "2023.3.0")

FetchContent_Declare(netcomm_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(netcomm_headers)

FetchContent_Declare(netcomm_libs URL ${LIB_URL})
FetchContent_MakeAvailable(netcomm_libs)

getniurl("runtime" "2023.3.0")

FetchContent_Declare(runtime_libs URL ${LIB_URL})
FetchContent_MakeAvailable(runtime_libs)

getniurl("visa" "2023.3.0")

FetchContent_Declare(visa_headers URL ${HEADER_URL})
FetchContent_MakeAvailable(visa_headers)

FetchContent_Declare(visa_libs URL ${LIB_URL})
FetchContent_MakeAvailable(visa_libs)

cmake_print_variables(chipobject_libs_SOURCE_DIR)
cmake_print_variables(PATH_SUFFIX)
find_library(
  CHIPOBJECT_LIBRARY
  NAMES "libRoboRIO_FRC_ChipObject.so.23.0.0"
  HINTS ${chipobject_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(CHIPOBJECT_HEADERS ${chipobject_libs_SOURCE_DIR})

find_library(
  NETCOMM_LIBRARY
  NAMES "libFRC_NetworkCommunication.so.23.0.0"
  HINTS ${netcomm_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(NETCOMM_HEADERS ${netcomm_libs_SOURCE_DIR})

find_library(
  FPGASHIM_LIBRARY
  NAMES "libfpgalvshim.so"
  HINTS ${runtime_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

find_library(
  CANSHIM_LIBRARY
  NAMES "libembcanshim.so"
  HINTS ${runtime_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

find_library(
  VISA_LIBRARY
  NAMES "libvisa.so.22.5.0"
  HINTS ${visa_libs_SOURCE_DIR}
  PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
  NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

set(VISA_HEADERS ${visa_libs_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  nilibraries
  DEFAULT_MSG
  CHIPOBJECT_HEADERS
  NETCOMM_HEADERS
  VISA_HEADERS
  CHIPOBJECT_LIBRARY
  NETCOMM_LIBRARY
  FPGASHIM_LIBRARY
  CANSHIM_LIBRARY
  VISA_LIBRARY)

mark_as_advanced(
  CHIPOBJECT_HEADERS
  NETCOMM_HEADERS
  VISA_HEADERS
  CHIPOBJECT_LIBRARY
  NETCOMM_LIBRARY
  FPGASHIM_LIBRARY
  CANSHIM_LIBRARY
  VISA_LIBRARY)

add_library(nilibraries INTERFACE)

if(NILIBRARIES_FOUND AND NOT TARGET nilibraries::nilibraries)
  add_library(chipobject::chipobject SHARED IMPORTED)

  set_target_properties(
    chipobject::chipobject
    PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${CHIPOBJECT_HEADERS}
               IMPORTED_LOCATION ${CHIPOBJECT_LIBRARY})

  add_library(netcomm::netcomm SHARED IMPORTED)

  set_target_properties(
    netcomm::netcomm PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${NETCOMM_HEADERS}
                                IMPORTED_LOCATION ${NETCOMM_LIBRARY})

  add_library(fpgashim::fpgashim SHARED IMPORTED)

  set_target_properties(fpgashim::fpgashim PROPERTIES IMPORTED_LOCATION
                                                      ${FPGASHIM_LIBRARY})

  add_library(canshim::canshim SHARED IMPORTED)

  set_target_properties(canshim::canshim PROPERTIES IMPORTED_LOCATION
                                                    ${CANSHIM_LIBRARY})

  add_library(visa::visa SHARED IMPORTED)

  set_target_properties(
    visa::visa PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${VISA_HEADERS}
                          IMPORTED_LOCATION ${VISA_LIBRARY})

  putlibsindeployfolder(${CHIPOBJECT_LIBRARY})
  putlibsindeployfolder(${NETCOMM_LIBRARY})
  putlibsindeployfolder(${FPGASHIM_LIBRARY})
  putlibsindeployfolder(${CANSHIM_LIBRARY})
  putlibsindeployfolder(${VISA_LIBRARY})

  target_link_libraries(
    nilibraries INTERFACE chipobject::chipobject netcomm::netcomm
                          canshim::canshim fpgashim::fpgashim visa::visa)
endif()
