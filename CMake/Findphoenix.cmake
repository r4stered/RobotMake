include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

if(NOT "${TOOLCHAIN_TRIPLE}" STREQUAL "arm-nilrt-linux-gnueabi")
    set(PHOENIX_SIM TRUE)
endif()

getctreurl("tools" "24.2.0" False)

fetchcontent_declare(tools_headers URL ${HEADER_URL})
fetchcontent_makeavailable(tools_headers)

fetchcontent_declare(tools_libs URL ${LIB_URL})
fetchcontent_makeavailable(tools_libs)

getctreurl("wpiapi-cpp" "24.2.0" False)

fetchcontent_declare(wpiapicpp_headers URL ${HEADER_URL})
fetchcontent_makeavailable(wpiapicpp_headers)

fetchcontent_declare(wpiapicpp_libs URL ${LIB_URL})
fetchcontent_makeavailable(wpiapicpp_libs)

if(PHOENIX_SIM)
    getctreurl("tools-sim" "24.2.0" True)

    fetchcontent_declare(toolssim_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(toolssim_headers)

    fetchcontent_declare(toolssim_libs URL ${LIB_URL})
    fetchcontent_makeavailable(toolssim_libs)

    getctreurl("wpiapi-cpp-sim" "24.2.0" True)

    fetchcontent_declare(wpiapicppsim_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(wpiapicppsim_headers)

    fetchcontent_declare(wpiapicppsim_libs URL ${LIB_URL})
    fetchcontent_makeavailable(wpiapicppsim_libs)

    getctreurl("simTalonSRX" "24.2.0" True)

    fetchcontent_declare(simtalonsrx_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simtalonsrx_headers)

    fetchcontent_declare(simtalonsrx_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simtalonsrx_libs)

    getctreurl("simTalonFX" "24.2.0" True)

    fetchcontent_declare(simtalonfx_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simtalonfx_headers)

    fetchcontent_declare(simtalonfx_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simtalonfx_libs)

    getctreurl("simVictorSPX" "24.2.0" True)

    fetchcontent_declare(simvictorspx_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simvictorspx_headers)

    fetchcontent_declare(simvictorspx_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simvictorspx_libs)

    getctreurl("simPigeonIMU" "24.2.0" True)

    fetchcontent_declare(simpigeonimu_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simpigeonimu_headers)

    fetchcontent_declare(simpigeonimu_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simpigeonimu_libs)

    getctreurl("simCANCoder" "24.2.0" True)

    fetchcontent_declare(simcancoder_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simcancoder_headers)

    fetchcontent_declare(simcancoder_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simcancoder_libs)

    getctreurl("simProTalonFX" "24.2.0" True)

    fetchcontent_declare(simprotalonfx_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simprotalonfx_headers)

    fetchcontent_declare(simprotalonfx_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simprotalonfx_libs)

    getctreurl("simProCANcoder" "24.2.0" True)

    fetchcontent_declare(simprocancoder_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simprocancoder_headers)

    fetchcontent_declare(simprocancoder_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simprocancoder_libs)

    getctreurl("simProPigeon2" "24.2.0" True)

    fetchcontent_declare(simpropigeon2_headers URL ${HEADER_URL})
    fetchcontent_makeavailable(simpropigeon2_headers)

    fetchcontent_declare(simpropigeon2_libs URL ${LIB_URL})
    fetchcontent_makeavailable(simpropigeon2_libs)
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(DEBUG_STRING "d")
else()
    set(DEBUG_STRING "")
endif()

if(WIN32)
    find_file(
        TOOLS_DLL
        NAMES "CTRE_PhoenixTools.dll"
        HINTS ${tools_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_file(
        WPIAPICPP_DLL
        NAMES "CTRE_Phoenix6_WPI.dll"
        HINTS ${wpiapicpp_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    if(PHOENIX_SIM)
        find_file(
            TOOLSSIM_DLL
            NAMES "CTRE_PhoenixTools_Sim.dll"
            HINTS ${toolssim_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            WPIAPICPPSIM_DLL
            NAMES "CTRE_Phoenix6_WPISim.dll"
            HINTS ${wpiapicppsim_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMTALONSRX_DLL
            NAMES "CTRE_SimTalonSRX.dll"
            HINTS ${simtalonsrx_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMTALONFX_DLL
            NAMES "CTRE_SimTalonFX.dll"
            HINTS ${simtalonfx_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMVICTORSPX_DLL
            NAMES "CTRE_SimVictorSPX.dll"
            HINTS ${simvictorspx_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMPIGEONIMU_DLL
            NAMES "CTRE_SimPigeonIMU.dll"
            HINTS ${simpigeonimu_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMCANCODER_DLL
            NAMES "CTRE_SimCANCoder.dll"
            HINTS ${simcancoder_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMPROTALONFX_DLL
            NAMES "CTRE_SimProTalonFX.dll"
            HINTS ${simprotalonfx_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMPROCANCODER_DLL
            NAMES "CTRE_SimProCANcoder.dll"
            HINTS ${simprocancoder_libs_SOURCE_DIR}
            PATH_SUFFIXES ${PATH_SUFFIX}
            REQUIRED
            NO_DEFAULT_PATH
            NO_CMAKE_FIND_ROOT_PATH
        )

        find_file(
            SIMPROPIGEON2_DLL
            NAMES "CTRE_SimProPigeon2.dll"
            HINTS ${simpropigeon2_libs_SOURCE_DIR}
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
    WPIAPICPP_LIBRARY
    NAMES "CTRE_Phoenix6_WPI"
    HINTS ${wpiapicpp_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if(PHOENIX_SIM)
    find_library(
        TOOLSSIM_LIBRARY
        NAMES "CTRE_PhoenixTools_Sim"
        HINTS ${toolssim_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        WPIAPICPPSIM_LIBRARY
        NAMES "CTRE_Phoenix6_WPISim"
        HINTS ${wpiapicppsim_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMTALONSRX_LIBRARY
        NAMES "CTRE_SimTalonSRX"
        HINTS ${simtalonsrx_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMTALONFX_LIBRARY
        NAMES "CTRE_SimTalonFX"
        HINTS ${simtalonfx_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMVICTORSPX_LIBRARY
        NAMES "CTRE_SimVictorSPX"
        HINTS ${simvictorspx_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMPIGEONIMU_LIBRARY
        NAMES "CTRE_SimPigeonIMU"
        HINTS ${simpigeonimu_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMCANCODER_LIBRARY
        NAMES "CTRE_SimCANCoder"
        HINTS ${simcancoder_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMPROTALONFX_LIBRARY
        NAMES "CTRE_SimProTalonFX"
        HINTS ${simprotalonfx_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMPROCANCODER_LIBRARY
        NAMES "CTRE_SimProCANcoder"
        HINTS ${simprocancoder_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )

    find_library(
        SIMPROPIGEON2_LIBRARY
        NAMES "CTRE_SimProPigeon2"
        HINTS ${simpropigeon2_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
endif()

set(TOOLS_HEADERS ${tools_headers_SOURCE_DIR})
set(WPIAPICPP_HEADERS ${wpiapicpp_headers_SOURCE_DIR})

if(PHOENIX_SIM)
    set(TOOLSSIM_HEADERS ${toolssim_headers_SOURCE_DIR})
    set(WPIAPICPPSIM_HEADERS ${wpiapicppsim_headers_SOURCE_DIR})
    set(SIMTALONSRX_HEADERS ${simtalonsrx_headers_SOURCE_DIR})
    set(SIMTALONFX_HEADERS ${simtalonfx_headers_SOURCE_DIR})
    set(SIMVICTORSPX_HEADERS ${simvictorspx_headers_SOURCE_DIR})
    set(SIMPIGEONIMU_HEADERS ${simpigeonimu_headers_SOURCE_DIR})
    set(SIMCANCODER_HEADERS ${simcancoder_headers_SOURCE_DIR})
    set(SIMPROTALONFX_HEADERS ${simprotalonfx_headers_SOURCE_DIR})
    set(SIMPROCANCODER_HEADERS ${simprocancoder_headers_SOURCE_DIR})
    set(SIMPROPIGEON2_HEADERS ${simpropigeon2_headers_SOURCE_DIR})
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
    phoenix
    DEFAULT_MSG
    TOOLS_HEADERS
    TOOLS_LIBRARY
    WPIAPICPP_HEADERS
    WPIAPICPP_LIBRARY
)

mark_as_advanced(
    TOOLS_HEADERS
    TOOLS_LIBRARY
    WPIAPICPP_HEADERS
    WPIAPICPP_LIBRARY
    WPIAPICPPSIM_HEADERS
    WPIAPICPPSIM_LIBRARY
    SIMTALONSRX_HEADERS
    SIMTALONSRX_LIBRARY
    SIMTALONFX_HEADERS
    SIMTALONFX_LIBRARY
    SIMVICTORSPX_HEADERS
    SIMVICTORSPX_LIBRARY
    SIMPIGEONIMU_HEADERS
    SIMPIGEONIMU_LIBRARY
    SIMCANCODER_HEADERS
    SIMCANCODER_LIBRARY
    SIMPROTALONFX_HEADERS
    SIMPROTALONFX_LIBRARY
    SIMPROCANCODER_HEADERS
    SIMPROCANCODER_LIBRARY
    SIMPROPIGEON2_HEADERS
    SIMPROPIGEON2_LIBRARY
)

add_library(phoenix INTERFACE)

if(PHOENIX_FOUND AND NOT TARGET phoenix::phoenix)
    add_library(tools::tools SHARED IMPORTED)
    add_library(wpiapicpp::wpiapicpp SHARED IMPORTED)

    if(PHOENIX_SIM)
        add_library(toolssim::toolssim SHARED IMPORTED)
        add_library(wpiapicppsim::wpiapicppsim SHARED IMPORTED)
        add_library(simtalonsrx::simtalonsrx SHARED IMPORTED)
        add_library(simtalonfx::simtalonfx SHARED IMPORTED)
        add_library(simvictorspx::simvictorspx SHARED IMPORTED)
        add_library(simpigeonimu::simpigeonimu SHARED IMPORTED)
        add_library(simcancoder::simcancoder SHARED IMPORTED)
        add_library(simprotalonfx::simprotalonfx SHARED IMPORTED)
        add_library(simprocancoder::simprocancoder SHARED IMPORTED)
        add_library(simpropigeon2::simpropigeon2 SHARED IMPORTED)
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
            wpiapicpp::wpiapicpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPP_HEADERS}
                IMPORTED_LOCATION ${WPIAPICPP_DLL}
                IMPORTED_IMPLIB ${WPIAPICPP_LIBRARY}
        )

        if(PHOENIX_SIM)
            set_target_properties(
                toolssim::toolssim
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${TOOLSSIM_HEADERS}
                    IMPORTED_LOCATION ${TOOLSSIM_DLL}
                    IMPORTED_IMPLIB ${TOOLSSIM_LIBRARY}
            )

            set_target_properties(
                wpiapicppsim::wpiapicppsim
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPPSIM_HEADERS}
                    IMPORTED_LOCATION ${WPIAPICPPSIM_DLL}
                    IMPORTED_IMPLIB ${WPIAPICPPSIM_LIBRARY}
            )

            set_target_properties(
                simtalonsrx::simtalonsrx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMTALONSRX_HEADERS}
                    IMPORTED_LOCATION ${SIMTALONSRX_DLL}
                    IMPORTED_IMPLIB ${SIMTALONSRX_LIBRARY}
            )

            set_target_properties(
                simtalonfx::simtalonfx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMTALONFX_HEADERS}
                    IMPORTED_LOCATION ${SIMTALONFX_DLL}
                    IMPORTED_IMPLIB ${SIMTALONFX_LIBRARY}
            )

            set_target_properties(
                simvictorspx::simvictorspx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMVICTORSPX_HEADERS}
                    IMPORTED_LOCATION ${SIMVICTORSPX_DLL}
                    IMPORTED_IMPLIB ${SIMVICTORSPX_LIBRARY}
            )

            set_target_properties(
                simpigeonimu::simpigeonimu
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPIGEONIMU_HEADERS}
                    IMPORTED_LOCATION ${SIMPIGEONIMU_DLL}
                    IMPORTED_IMPLIB ${SIMPIGEONIMU_LIBRARY}
            )

            set_target_properties(
                simcancoder::simcancoder
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMCANCODER_HEADERS}
                    IMPORTED_LOCATION ${SIMCANCODER_DLL}
                    IMPORTED_IMPLIB ${SIMCANCODER_LIBRARY}
            )

            set_target_properties(
                simprotalonfx::simprotalonfx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROTALONFX_HEADERS}
                    IMPORTED_LOCATION ${SIMPROTALONFX_DLL}
                    IMPORTED_IMPLIB ${SIMPROTALONFX_LIBRARY}
            )

            set_target_properties(
                simprocancoder::simprocancoder
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROCANCODER_HEADERS}
                    IMPORTED_LOCATION ${SIMPROCANCODER_DLL}
                    IMPORTED_IMPLIB ${SIMPROCANCODER_LIBRARY}
            )

            set_target_properties(
                simpropigeon2::simpropigeon2
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROPIGEON2_HEADERS}
                    IMPORTED_LOCATION ${SIMPROPIGEON2_DLL}
                    IMPORTED_IMPLIB ${SIMPROPIGEON2_LIBRARY}
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
            wpiapicpp::wpiapicpp
            PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPP_HEADERS}
                IMPORTED_LOCATION ${WPIAPICPP_LIBRARY}
        )

        putlibsindeployfolder(${TOOLS_LIBRARY})
        putlibsindeployfolder(${WPIAPICPP_LIBRARY})

        if(PHOENIX_SIM)
            set_target_properties(
                toolssim::toolssim
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${TOOLSSIM_HEADERS}
                    IMPORTED_LOCATION ${TOOLSSIM_LIBRARY}
            )
            set_target_properties(
                wpiapicppsim::wpiapicppsim
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${WPIAPICPPSIM_HEADERS}
                    IMPORTED_LOCATION ${WPIAPICPPSIM_LIBRARY}
            )
            set_target_properties(
                simtalonsrx::simtalonsrx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMTALONSRX_HEADERS}
                    IMPORTED_LOCATION ${SIMTALONSRX_LIBRARY}
            )
            set_target_properties(
                simtalonfx::simtalonfx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMTALONFX_HEADERS}
                    IMPORTED_LOCATION ${SIMTALONFX_LIBRARY}
            )
            set_target_properties(
                simvictorspx::simvictorspx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMVICTORSPX_HEADERS}
                    IMPORTED_LOCATION ${SIMVICTORSPX_LIBRARY}
            )
            set_target_properties(
                simpigeonimu::simpigeonimu
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPIGEONIMU_HEADERS}
                    IMPORTED_LOCATION ${SIMPIGEONIMU_LIBRARY}
            )
            set_target_properties(
                simcancoder::simcancoder
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMCANCODER_HEADERS}
                    IMPORTED_LOCATION ${SIMCANCODER_LIBRARY}
            )
            set_target_properties(
                simprotalonfx::simprotalonfx
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROTALONFX_HEADERS}
                    IMPORTED_LOCATION ${SIMPROTALONFX_LIBRARY}
            )
            set_target_properties(
                simprocancoder::simprocancoder
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROCANCODER_HEADERS}
                    IMPORTED_LOCATION ${SIMPROCANCODER_LIBRARY}
            )
            set_target_properties(
                simpropigeon2::simpropigeon2
                PROPERTIES
                    INTERFACE_INCLUDE_DIRECTORIES ${SIMPROPIGEON2_HEADERS}
                    IMPORTED_LOCATION ${SIMPROPIGEON2_LIBRARY}
            )
        endif()
    endif()
    if(PHOENIX_SIM)
        target_link_libraries(
            phoenix
            INTERFACE
                toolssim::toolssim
                wpiapicppsim::wpiapicppsim
                simtalonsrx::simtalonsrx
                simtalonfx::simtalonfx
                simvictorspx::simvictorspx
                simpigeonimu::simpigeonimu
                simcancoder::simcancoder
                simprotalonfx::simprotalonfx
                simprocancoder::simprocancoder
                simpropigeon2::simpropigeon2
        )
    else()
        target_link_libraries(phoenix INTERFACE tools::tools wpiapicpp::wpiapicpp)
    endif()
endif()
