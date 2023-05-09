include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

if(NOT "${TOOLCHAIN_TRIPLE}" STREQUAL "arm-nilrt-linux-gnueabi")
    set(PHOENIX_SIM TRUE)
endif()

GetCtreUrl("tools" "23.0.12" FALSE)

FetchContent_Declare(
  tools_headers
  URL ${HEADER_URL}
)

FetchContent_Declare(
  tools_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(tools_headers)

FetchContent_Declare(
  tools_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(tools_libs)

GetCtreUrl("api-cpp" "5.30.4" FALSE)

FetchContent_Declare(
  apicpp_headers
  URL ${HEADER_URL}
)

FetchContent_Declare(
  apicpp_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(apicpp_headers)

FetchContent_Declare(
  apicpp_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(apicpp_libs)

GetCtreUrl("cci" "5.30.4" False)

FetchContent_Declare(
  cci_headers
  URL ${HEADER_URL}
)

FetchContent_Declare(
  cci_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(cci_headers)

FetchContent_Declare(
  cci_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(cci_libs)

GetCtreUrl("wpiapi-cpp" "5.30.4" False)

FetchContent_Declare(
  wpiapicpp_headers
  URL ${HEADER_URL}
)
FetchContent_MakeAvailable(wpiapicpp_headers)

FetchContent_Declare(
  wpiapicpp_libs
  URL ${LIB_URL}
)
FetchContent_MakeAvailable(wpiapicpp_libs)

if(PHOENIX_SIM)
    GetCtreUrl("api-cpp-sim" "5.30.4" True)

    FetchContent_Declare(
      apicppsim_headers
      URL ${HEADER_URL}
    )
    FetchContent_MakeAvailable(apicppsim_headers)

    FetchContent_Declare(
      apicppsim_libs
      URL ${LIB_URL}
    )
    FetchContent_MakeAvailable(apicppsim_libs)

    GetCtreUrl("cci-sim" "5.30.4" True)

    FetchContent_Declare(
        ccisim_headers
        URL ${HEADER_URL}
    )

    FetchContent_Declare(
        ccisim_headers
        URL ${HEADER_URL}
    )
    FetchContent_MakeAvailable(ccisim_headers)

    FetchContent_Declare(
        ccisim_libs
        URL ${LIB_URL}
    )
    FetchContent_MakeAvailable(ccisim_libs)

    GetCtreUrl("wpiapi-cpp-sim" "5.30.4" True)

    FetchContent_Declare(
        wpiapicppsim_headers
        URL ${HEADER_URL}
    )

    FetchContent_Declare(
        wpiapicppsim_headers
        URL ${HEADER_URL}
    )
    FetchContent_MakeAvailable(wpiapicppsim_headers)

    FetchContent_Declare(
        wpiapicppsim_libs
        URL ${LIB_URL}
    )
    FetchContent_MakeAvailable(wpiapicppsim_libs)
endif()


if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(DEBUG_STRING "d")
else()
  set(DEBUG_STRING "")
endif()

if(WIN32)
  find_file(TOOLS_DLL
    NAMES "CTRE_PhoenixTools.dll"
    HINTS ${tools_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )

  find_file(APICPP_DLL
    NAMES "CTRE_Phoenix.dll"
    HINTS ${apicpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )

  find_file(CCI_DLL
    NAMES "CTRE_PhoenixCCI.dll"
    HINTS ${cci_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )

  find_file(WPIAPICPP_DLL
    NAMES "CTRE_Phoenix_WPI.dll"
    HINTS ${wpiapicpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )

  if(PHOENIX_SIM)
    find_file(APICPPSIM_DLL
      NAMES "CTRE_PhoenixSim.dll"
      HINTS ${apicppsim_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(CCISIM_DLL
      NAMES "CTRE_PhoenixCCISim.dll"
      HINTS ${ccisim_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(WPIAPICPPSIM_DLL
      NAMES "CTRE_Phoenix_WPISim.dll"
      HINTS ${wpiapicppsim_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
    )
  endif()
endif()

find_library(
    TOOLS_LIBRARY
    NAMES "CTRE_PhoenixTools"
    HINTS ${tools_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    APICPP_LIBRARY
    NAMES "CTRE_Phoenix"
    HINTS ${apicpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    CCI_LIBRARY
    NAMES "CTRE_PhoenixCCI"
    HINTS ${cci_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

find_library(
    WPIAPICPP_LIBRARY
    NAMES "CTRE_Phoenix_WPI"
    HINTS ${wpiapicpp_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if(PHOENIX_SIM)
  find_library(
    APICPPSIM_LIBRARY
    NAMES "CTRE_PhoenixSim"
    HINTS ${apicppsim_libs_SOURCE_DIR} 
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )

  find_library(
      CCISIM_LIBRARY
      NAMES "CTRE_PhoenixCCISim"
      HINTS ${ccisim_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
  )

  find_library(
      WPIAPICPPSIM_LIBRARY
      NAMES "CTRE_Phoenix_WPISim"
      HINTS ${wpiapicppsim_libs_SOURCE_DIR} 
      PATH_SUFFIXES ${PATH_SUFFIX}
      REQUIRED
      NO_DEFAULT_PATH
      NO_CMAKE_FIND_ROOT_PATH
  )
endif()

set(TOOLS_HEADERS ${tools_headers_SOURCE_DIR})
set(APICPP_HEADERS ${apicpp_headers_SOURCE_DIR})
set(CCI_HEADERS ${cci_headers_SOURCE_DIR})
set(WPIAPICPP_HEADERS ${wpiapicpp_headers_SOURCE_DIR})

if(PHOENIX_SIM)
  set(APICPPSIM_HEADERS ${apicppsim_headers_SOURCE_DIR})
  set(CCISIM_HEADERS ${ccisim_headers_SOURCE_DIR})
  set(WPIAPICPPSIM_HEADERS ${wpiapicppsim_headers_SOURCE_DIR})
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  phoenix 
  DEFAULT_MSG
  TOOLS_HEADERS
  TOOLS_LIBRARY
  APICPP_HEADERS
  APICPP_LIBRARY
  CCI_HEADERS
  CCI_LIBRARY
)

mark_as_advanced(  
  TOOLS_HEADERS
  TOOLS_LIBRARY
  APICPP_HEADERS
  APICPP_LIBRARY
  CCI_HEADERS
  CCI_LIBRARY
  WPIAPICPP_HEADERS
  WPIAPICPP_LIBRARY
  APICPPSIM_HEADERS
  APICPPSIM_LIBRARY
  CCISIM_HEADERS
  CCISIM_LIBRARY
  WPIAPICPPSIM_HEADERS
  WPIAPICPPSIM_LIBRARY
)

add_library(phoenix INTERFACE)

if(PHOENIX_FOUND AND NOT TARGET phoenix::phoenix)

    add_library(tools::tools SHARED IMPORTED)
    add_library(apicpp::apicpp SHARED IMPORTED)
    add_library(cci::cci SHARED IMPORTED)
    add_library(wpiapicpp::wpiapicpp SHARED IMPORTED)

    if(PHOENIX_SIM)
      add_library(apicppsim::apicppsim SHARED IMPORTED)
      add_library(ccisim::ccisim SHARED IMPORTED)
      add_library(wpiapicppsim::wpiapicppsim SHARED IMPORTED)
    endif()

    if(WIN32)
        set_target_properties(
            tools::tools
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${TOOLS_HEADERS}
                IMPORTED_LOCATION ${TOOLS_DLL}
                IMPORTED_IMPLIB ${TOOLS_LIBRARY}
        )
        set_target_properties(
            apicpp::apicpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${APICPP_HEADERS}
                IMPORTED_LOCATION ${APICPP_DLL}
                IMPORTED_IMPLIB ${APICPP_LIBRARY}
        )
        set_target_properties(
            cci::cci
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${CCI_HEADERS}
                IMPORTED_LOCATION ${CCI_DLL}
                IMPORTED_IMPLIB ${CCI_LIBRARY}
        )

        set_target_properties(
          wpiapicpp::wpiapicpp
          PROPERTIES
              INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPP_HEADERS}
              IMPORTED_LOCATION ${WPIAPICPP_DLL}
              IMPORTED_IMPLIB ${WPIAPICPP_LIBRARY}
        )

        if(PHOENIX_SIM)
          set_target_properties(
            apicppsim::apicppsim
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${APICPPSIM_HEADERS}
                IMPORTED_LOCATION ${APICPPSIM_DLL}
                IMPORTED_IMPLIB ${APICPPSIM_LIBRARY}
          )

          set_target_properties(
              ccisim::ccisim
              PROPERTIES
                  INTERFACE_INCLUDE_DIRECTORIES ${CCISIM_HEADERS}
                  IMPORTED_LOCATION ${CCISIM_DLL}
                  IMPORTED_IMPLIB ${CCISIM_LIBRARY}
          )

          set_target_properties(
              wpiapicppsim::wpiapicppsim
              PROPERTIES
                  INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPPSIM_HEADERS}
                  IMPORTED_LOCATION ${WPIAPICPPSIM_DLL}
                  IMPORTED_IMPLIB ${WPIAPICPPSIM_LIBRARY}
          )          
        endif()
    else()
      set_target_properties(
        tools::tools
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${TOOLS_HEADERS}
          IMPORTED_LOCATION ${TOOLS_LIBRARY}
        )
      set_target_properties(
        apicpp::apicpp
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${APICPP_HEADERS}
          IMPORTED_LOCATION ${APICPP_LIBRARY}
        )
      set_target_properties(
        cci::cci
        PROPERTIES
          INTERFACE_INCLUDE_DIRECTORIES ${CCI_HEADERS}
          IMPORTED_LOCATION ${CCI_LIBRARY}
        )
      set_target_properties(
          wpiapicpp::wpiapicpp
          PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPP_HEADERS}
            IMPORTED_LOCATION ${WPIAPICPP_LIBRARY}
        )      

      if(PHOENIX_SIM)
        set_target_properties(
          apicppsim::apicppsim
          PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${APICPPSIM_HEADERS}
            IMPORTED_LOCATION ${APICPPSIM_LIBRARY}
        )

        set_target_properties(
          ccisim::ccisim
          PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${CCISIM_HEADERS}
            IMPORTED_LOCATION ${CCISIM_LIBRARY}
          )

        set_target_properties(
          wpiapicppsim::wpiapicppsim
          PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPPSIM_HEADERS}
            IMPORTED_LOCATION ${WPIAPICPPSIM_LIBRARY}
          )
      endif()
    endif()
    if(PHOENIX_SIM)
      target_link_libraries(phoenix INTERFACE tools::tools apicpp::apicpp cci::cci wpiapicpp::wpiapicpp apicppsim::apicppsim ccisim::ccisim wpiapicppsim::wpiapicppsim)
    else()
      target_link_libraries(phoenix INTERFACE tools::tools apicpp::apicpp cci::cci wpiapicpp::wpiapicpp)
    endif()
endif()