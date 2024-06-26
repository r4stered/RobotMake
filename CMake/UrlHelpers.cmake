include(CMakePrintHelpers)
include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/Versions.cmake)

# This function sets OS_STRING equal to the appropriate url string for downloading wpi deps
function(GetOsNameForUrl)
    set(OS_STRING ${CMAKE_SYSTEM_NAME})
    if(OS_STRING STREQUAL "Darwin")
        set(OS_STRING "macos")
    endif()
    string(TOLOWER ${OS_STRING} OS_STRING)  

    return(PROPAGATE OS_STRING)
endfunction(GetOsNameForUrl)

# This function sets BUILD_TYPE_STRING equal to the appropriate url string for downloading wpi deps
function(GetBuildTypeForUrl)
    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(BUILD_TYPE_STRING "debug")
    else()
        set(BUILD_TYPE_STRING "")
    endif()

    return(PROPAGATE BUILD_TYPE_STRING)
endfunction(GetBuildTypeForUrl)

# This function sets ARCH_STRING equal to the appropriate url string for downloading wpi deps
function(GetArchStringForUrl)
    # If we are building locally, we won't have a toolchain file defined
    if(NOT DEFINED TOOLCHAIN_TRIPLE)
        if(${CMAKE_SYSTEM_NAME} STREQUAL "Darwin")
            set(ARCH_STRING "universal")
        # Asssume for now we are building from x86 if local. TODO: Handle local arm builds
        else()
            set(ARCH_STRING "x86-64")
        endif()
    else()
        # Rio build
        if(${TOOLCHAIN_TRIPLE} STREQUAL "arm-nilrt-linux-gnueabi")
            set(ARCH_STRING "athena")
        # Generic arm build
        elseif(${TOOLCHAIN_TRIPLE} STREQUAL "aarch64-none-linux-gnu-")
            set(ARCH_STRING "arm64")
        endif()
    endif()

    return(PROPAGATE ARCH_STRING)
endfunction(GetArchStringForUrl)

# This function sets SHARED_STRING equal to the appropriate url string for downloading wpi deps
function(GetSharedOrStaticStringForUrl is_shared)
    if(${is_shared})
        set(SHARED_STRING "")
    else()
        set(SHARED_STRING "static")
    endif()

    return(PROPAGATE SHARED_STRING)
endfunction(GetSharedOrStaticStringForUrl)

# Gets the header and lib url for CTRE libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetCtreUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    string(FIND "${library_name}" "sim" FOUND_SIM)

    if(${FOUND_SIM} EQUAL -1)
        set(BASE_URL "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/${library_name}/${version}")
    else()
        set(BASE_URL "https://maven.ctr-electronics.com/release/com/ctre/phoenix6/sim/${library_name}/${version}")
    endif()

    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    #Special case for wierd capitilization of ctre sim libs
    if(${library_name} STREQUAL "simtalonsrx")
        string(REPLACE "simtalonsrx" "simTalonSRX" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simtalonsrx" "simTalonSRX" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simtalonfx")
        string(REPLACE "simtalonfx" "simTalonFX" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simtalonfx" "simTalonFX" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simvictorspx")
        string(REPLACE "simvictorspx" "simVictorSPX" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simvictorspx" "simVictorSPX" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simpigeonimu")
        string(REPLACE "simpigeonimu" "simPigeonIMU" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simpigeonimu" "simPigeonIMU" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simcancoder")
        string(REPLACE "simcancoder" "simCANCoder" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simcancoder" "simCANCoder" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simprotalonfx")
        string(REPLACE "simprotalonfx" "simProTalonFX" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simprotalonfx" "simProTalonFX" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simprocancoder")
        string(REPLACE "simprocancoder" "simProCANcoder" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simprocancoder" "simProCANcoder" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    if(${library_name} STREQUAL "simpropigeon2")
        string(REPLACE "simpropigeon2" "simProPigeon2" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "simpropigeon2" "simProPigeon2" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()


    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetCtreUrl)

# Gets the header and lib url for the Rev libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetRevUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # We need to make the library name unique because rev and playing with fusion use cpp and driver lib names
    string(REPLACE "revlib_" "" url_library_name ${library_name})

    set(BASE_URL "https://maven.revrobotics.com/com/revrobotics/frc/REVLib-${url_library_name}/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/REVLib-${url_library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/REVLib-${url_library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetRevUrl)

# Gets the header and lib url for the Playing with Fusion libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetPlayingWithFusionUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # We need to make the library name unique because rev and playing with fusion use cpp and driver lib names
    string(REPLACE "playingwithfusion_" "" url_library_name ${library_name})

    set(BASE_URL "https://www.playingwithfusion.com/frc/maven/com/playingwithfusion/frc/PlayingWithFusion-${url_library_name}/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/PlayingWithFusion-${url_library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/PlayingWithFusion-${url_library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetPlayingWithFusionUrl)

# Gets the header and lib url for the Photonvision libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetPhotonvisionUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    set(BASE_URL "https://maven.photonvision.org/repository/internal/org/photonvision/${library_name}-cpp/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetPhotonvisionUrl)

# Gets the header and lib url for the Pathplanner libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetPathplannerLibUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    set(BASE_URL "https://github.com/3015RangerRobotics/3015RangerRobotics.github.io/raw/main/${library_name}/repo/com/pathplanner/lib/${library_name}-cpp/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    #Special case for wierd capitilization of PathplannerLib
    if(${library_name} STREQUAL "pathplannerlib")
        string(REPLACE "pathplannerlib-cpp" "PathplannerLib-cpp" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "pathplannerlib-cpp" "PathplannerLib-cpp" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetPathplannerLibUrl)

# Gets the header and lib url for the Choreo libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetChoreoLibUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    set(BASE_URL "https://github.com/SleipnirGroup/${library_name}/raw/main/dep/com/choreo/lib/${library_name}-cpp/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    #Special case for wierd capitilization of Choreo
    if(${library_name} STREQUAL "choreolib")
        string(REPLACE "choreolib" "ChoreoLib" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "choreolib" "ChoreoLib" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetChoreoLibUrl)

# Gets the header and lib url for the Studica Labs NavX libraries.
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetStudicaLabsUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    string(SUBSTRING ${version} 0 4 STUDICA_LABS_YEAR)

    set(BASE_URL "https://dev.studica.com/maven/release/${STUDICA_LABS_YEAR}/com/kauailabs/${library_name}/frc/${library_name}-frc-cpp/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-frc-cpp-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-frc-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetStudicaLabsUrl)

# Gets the header and lib url for the roborio libraries. All of these are only shared and are only valid when building
# for the roborio
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetHalExtUrl library_name)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/halsim/${library_name}/${WPI_VERSION}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-${WPI_VERSION}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-${WPI_VERSION}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetHalExtUrl)

# Gets the header and lib url for the roborio libraries. All of these are only shared and are only valid when building
# for the roborio
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetNiUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries/${library_name}/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    set(LINK_TYPE_STRING "shared")
    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetNiUrl)

# Gets the header and lib url for a third party wpi packages. All of these are only static except opencv
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetThirdpartyUrl library_name version)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # These libs are avaliable as static only, so force it
    if(${library_name} STREQUAL "apriltaglib" OR
       ${library_name} STREQUAL "googletest" OR
       ${library_name} STREQUAL "imgui" OR
       ${library_name} STREQUAL "libssh"
    )
        set(SHARED_STRING "static")
    endif()


    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/frc${WPI_YEAR}/${library_name}/${version}")
    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-${version}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    if(${library_name} STREQUAL "opencv")
        set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/frc${WPI_YEAR}/${library_name}/${library_name}-cpp/${version}")
        set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${version}-headers.zip")
        set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${version}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    set(${library_name}_PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}")

    # Libssh has the os in the subdirectories
    if("${library_name}" STREQUAL "libssh" OR
       "${library_name}" STREQUAL "opencv")
        set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")
    endif()

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetThirdpartyUrl)

# Gets the header and lib url for a wpi package. You must make sure GET_SHARED_LIBS is set to True or False before calling this
# Returns 
    # ${library_name}_HEADER_URL - the url to the headers of the library
    # ${library_name}_LIB_URL - the url to the lib of the library
    # ${library_name}_PATH_SUFFIX - the subdirectory where the library files are located in the _deps folder
function(GetWpiUrl library_name)
    GetOsNameForUrl()
    GetBuildTypeForUrl()
    GetArchStringForUrl()
    GetSharedOrStaticStringForUrl(${GET_SHARED_LIBS})

    # These libs are avaliable as static only, so force it
    if(${library_name} STREQUAL "wpigui")
        set(SHARED_STRING "static")
    endif()

    set(BASE_URL "https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/${library_name}/${library_name}-cpp/${WPI_VERSION}")

    set(${library_name}_HEADER_URL "${BASE_URL}/${library_name}-cpp-${WPI_VERSION}-headers.zip")
    set(${library_name}_LIB_URL "${BASE_URL}/${library_name}-cpp-${WPI_VERSION}-${OS_STRING}${ARCH_STRING}${SHARED_STRING}${BUILD_TYPE_STRING}.zip")

    #Special case for wierd capitilization of commands v2
    if(${library_name} STREQUAL "wpilibnewcommands")
        string(REPLACE "wpilibnewcommands" "wpilibNewCommands" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "wpilibnewcommands" "wpilibNewCommands" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    #Special case for wierd capitilization of field images
    if(${library_name} STREQUAL "fieldimages")
        string(REPLACE "fieldimages" "fieldImages" ${library_name}_HEADER_URL ${${library_name}_HEADER_URL})
        string(REPLACE "fieldimages" "fieldImages" ${library_name}_LIB_URL ${${library_name}_LIB_URL})
    endif()

    # Sets the subdirectory to search in when calling find_library and similar functions
    if("${SHARED_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()
    set(${library_name}_PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}")

    return(PROPAGATE ${library_name}_HEADER_URL ${library_name}_LIB_URL ${library_name}_PATH_SUFFIX)
endfunction(GetWpiUrl)
