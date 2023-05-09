include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

GetThirdPartyLibraryUrl("opencv" "4.6.0-4")

FetchContent_Declare(
  opencv_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(opencv_headers)

FetchContent_Declare(
  opencv_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(opencv_libs)

set(OPENCV_LIST_OF_LIBS "")
if(GET_SHARED_LIBS)
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_aruco460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_calib3d460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_core460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_features2d460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_flann460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_gapi460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_highgui460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_imgcodecs460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_imgproc460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_ml460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_objdetect460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_photo460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_stitching460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_video460")
  list(APPEND OPENCV_LIST_OF_LIBS "opencv_videoio460")
else()
  list(APPEND OPENCV_LIST_OF_LIBS "opencv460")
endif()

#Stupid suffix conditions depending on OS
if(TOOLCHAIN_TRIPLE STREQUAL "arm-nilrt-linux-gnueabi")
  list(TRANSFORM OPENCV_LIST_OF_LIBS REPLACE "460" "")
  list(APPEND CMAKE_FIND_LIBRARY_SUFFIXES .so.4.6)
endif()

cmake_print_variables(OPENCV_LIST_OF_LIBS)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

if(WIN32)
  foreach(lib ${OPENCV_LIST_OF_LIBS})
    find_file(${lib}_CV_DLL
      NAMES ${lib}${DEBUG_STRING}.dll
      HINTS ${opencv_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
    )
  endforeach()
endif()

foreach(lib ${OPENCV_LIST_OF_LIBS})
  find_library(${lib}_CV_LIBRARY 
    NAMES ${lib}${DEBUG_STRING} 
    HINTS ${opencv_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endforeach()

set(OPENCV_HEADERS ${opencv_headers_SOURCE_DIR})

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  opencv 
  DEFAULT_MSG
  OPENCV_HEADERS
)

mark_as_advanced(OPENCV_HEADERS)

add_library(opencv INTERFACE)

if(OPENCV_FOUND AND NOT TARGET opencv::opencv)
  foreach(lib ${OPENCV_LIST_OF_LIBS})
    if(GET_SHARED_LIBS)
      add_library(${lib}::${lib} SHARED IMPORTED)
    else()
      add_library(${lib}::${lib} STATIC IMPORTED)
    endif()

    if(WIN32)
      set_target_properties(
        ${lib}::${lib}
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${OPENCV_HEADERS}
          IMPORTED_LOCATION ${${lib}_CV_DLL}
          IMPORTED_IMPLIB ${${lib}_CV_LIBRARY}
      )
    else()
      set_target_properties(
        ${lib}::${lib}
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${OPENCV_HEADERS}
          IMPORTED_LOCATION ${${lib}_CV_LIBRARY}
      )
    endif()
  target_link_libraries(opencv INTERFACE ${lib}::${lib})
  endforeach()
endif()