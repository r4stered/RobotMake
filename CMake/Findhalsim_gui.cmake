include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/UrlHelpers.cmake)

gethalexturl("halsim_gui" "2024.3.2")

fetchcontent_declare(halsim_gui_libs URL ${LIB_URL})
fetchcontent_makeavailable(halsim_gui_libs)

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
    set(DEBUG_STRING "d")
else()
    set(DEBUG_STRING "")
endif()

if(WIN32)
    find_file(
        HALSIM_GUI_DLL
        NAMES "halsim_gui${DEBUG_STRING}.dll"
        HINTS ${halsim_gui_libs_SOURCE_DIR}
        PATH_SUFFIXES ${PATH_SUFFIX}
        REQUIRED
        NO_DEFAULT_PATH
        NO_CMAKE_FIND_ROOT_PATH
    )
endif()

find_library(
    HALSIM_GUI_LIBRARY
    NAMES "halsim_gui${DEBUG_STRING}"
    HINTS ${halsim_gui_libs_SOURCE_DIR}
    PATH_SUFFIXES ${PATH_SUFFIX}
    REQUIRED
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(halsim_gui DEFAULT_MSG HALSIM_GUI_LIBRARY)

mark_as_advanced(HALSIM_GUI_LIBRARY)

if(HALSIM_GUI_FOUND AND NOT TARGET halsim_gui::halsim_gui)
    if(GET_SHARED_LIBS)
        add_library(halsim_gui::halsim_gui SHARED IMPORTED)
    else()
        add_library(halsim_gui::halsim_gui STATIC IMPORTED)
    endif()

    if(WIN32)
        set_target_properties(
            halsim_gui::halsim_gui
            PROPERTIES IMPORTED_LOCATION ${HALSIM_GUI_DLL} IMPORTED_IMPLIB ${HALSIM_GUI_LIBRARY}
        )
    else()
        set_target_properties(
            halsim_gui::halsim_gui
            PROPERTIES IMPORTED_LOCATION ${HALSIM_GUI_LIBRARY}
        )

        putlibsindeployfolder(${HALSIM_GUI_LIBRARY})
    endif()
endif()
