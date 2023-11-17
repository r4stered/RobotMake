function(FindWpiPackage packageName version install_folder)

  include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/DeployUtils.cmake)
  include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

  getwpiurl(${packageName} ${version})

  if(NOT "${install_folder}" STREQUAL "")
    set(INSTALL_FOLDER_STR ${install_folder})
  else()
    set(INSTALL_FOLDER_STR "")
  endif()

  FetchContent_Declare(
    ${packageName}_headers
    DOWNLOAD_COMMAND
      URL ${HEADER_URL}
    SOURCE_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-src/${INSTALL_FOLDER_STR}
    BINARY_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-build/${INSTALL_FOLDER_STR}
    SUBBUILD_DIR
      ${CMAKE_CURRENT_BINARY_DIR}/_deps/${packageName}_headers-subbuild/${INSTALL_FOLDER_STR}
  )

  FetchContent_MakeAvailable(${packageName}_headers)

  if(NOT "${install_folder}" STREQUAL "")
    cmake_path(GET ${packageName}_headers_SOURCE_DIR PARENT_PATH
               ${packageName}_FIXED_PATH)
    set(${packageName}_headers_SOURCE_DIR ${${packageName}_FIXED_PATH})
  endif()

  FetchContent_Declare(${packageName}_libs URL ${LIB_URL})

  FetchContent_MakeAvailable(${packageName}_libs)

  if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(DEBUG_STRING "d")
  else()
    set(DEBUG_STRING "")
  endif()

  if(${packageName} STREQUAL "hal")
    set(LIB_PREFIX "wpi")
  else()
    set(LIB_PREFIX "")
  endif()

  if(WIN32)
    find_file(
      ${packageName}_DLL
      NAMES ${LIB_PREFIX}${packageName}${DEBUG_STRING}.dll
      HINTS ${${packageName}_libs_SOURCE_DIR}
      PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
      NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
  endif()


  if(${packageName} STREQUAL "hal")
    set(REAL_LIB_NAME "Hal")
  elseif(${packageName} STREQUAL "wpilibnewcommands")
    set(REAL_LIB_NAME "wpilibNewCommands")
  else()
    set(REAL_LIB_NAME ${packageName})
  endif()

  find_library(
    ${packageName}_LIBRARY
    NAMES ${LIB_PREFIX}${REAL_LIB_NAME}${DEBUG_STRING}
    HINTS ${${packageName}_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX} REQUIRED
    NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)

  set(${packageName}_HEADERS ${${packageName}_headers_SOURCE_DIR})

  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(
    ${packageName} DEFAULT_MSG ${packageName}_LIBRARY ${packageName}_HEADERS)

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
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                   IMPORTED_LOCATION ${${packageName}_DLL}
                   IMPORTED_IMPLIB ${${packageName}_LIBRARY})
    else()
      set_target_properties(
        ${packageName}::${packageName}
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${${packageName}_HEADERS}"
                   IMPORTED_LOCATION ${${packageName}_LIBRARY})

      putlibsindeployfolder(${${packageName}_LIBRARY})
    endif()
  endif()

endfunction(FindWpiPackage)
