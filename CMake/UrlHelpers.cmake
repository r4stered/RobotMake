include(CMakePrintHelpers)

function(GetRevUrl library_name version)
    GetWpiUrlBase("https://maven.revrobotics.com/com/revrobotics/frc/REVLib-${library_name}" ${library_name} ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetPhotonVisionUrl version)
    GetWpiUrlBase("https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-cpp" "PhotonLib-cpp" ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetPathplannerUrl version)
    GetWpiUrlBase("https://github.com/3015RangerRobotics/3015RangerRobotics.github.io/raw/main/pathplannerlib/repo/com/pathplanner/lib/PathplannerLib-cpp" "PathplannerLib-cpp" ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetKauaiLabsUrl version)
    GetWpiUrlBase("https://dev.studica.com/maven/release/2023/com/kauailabs/navx/frc/navx-frc-cpp" "navx-frc-cpp" ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetNiUrl library_name version)
    GetWpiUrlBase("https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/ni-libraries" ${library_name} ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetWpiUrl library_name version)
    GetWpiUrlBase("https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first" ${library_name} ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetThirdPartyLibraryUrl library_name version)
    GetWpiUrlBase("https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/frc2023" ${library_name} ${version})
    set(HEADER_URL ${HEADER_URL} PARENT_SCOPE)
    set(LIB_URL ${LIB_URL} PARENT_SCOPE)
    set(PATH_SUFFIX ${PATH_SUFFIX} PARENT_SCOPE)
endfunction()

function(GetWpiUrlBase base_url_string library_name version)
    set(OS_STRING ${CMAKE_SYSTEM_NAME})
    string(TOLOWER ${OS_STRING} OS_STRING)

    if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
        set(BUILD_TYPE_STRING "debug")
    else()
        set(BUILD_TYPE_STRING "")
    endif()

    if(NOT DEFINED TOOLCHAIN_TRIPLE)
        if(OS_STRING STREQUAL "Darwin")
            set(OS_STRING "macos")
            set(ARCH_STRING "universal")
        else()
            set(ARCH_STRING "x86-64")
        endif()
    else()
        if(${TOOLCHAIN_TRIPLE} STREQUAL "arm-nilrt-linux-gnueabi")
            set(ARCH_STRING "athena")
        endif()
    endif()

    if(GET_SHARED_LIBS)
        set(STATIC_STRING "")
    else()
        set(STATIC_STRING "static")
    endif()

    if(${library_name} STREQUAL "wpilibnewcommands")
        set(library_name "wpilibNewCommands")
    endif()

    if(${library_name} STREQUAL "apriltaglib")
        #April tag lib only has static binaries avaliable
        set(STATIC_STRING "static")
        set(BASE_URL "${base_url_string}/${library_name}/${version}/${library_name}-${version}-")
    elseif(${library_name} STREQUAL "chipobject" OR 
           ${library_name} STREQUAL "netcomm" OR
           ${library_name} STREQUAL "runtime" OR
           ${library_name} STREQUAL "visa")
        #NI libs are only avaliable as shared libs
        set(STATIC_STRING "")
        set(BASE_URL "${base_url_string}/${library_name}/${version}/${library_name}-${version}-")
    elseif(${library_name} STREQUAL "navx-frc-cpp" OR 
           ${library_name} STREQUAL "PathplannerLib-cpp" OR
           ${library_name} STREQUAL "PhotonLib-cpp")
        set(BASE_URL "${base_url_string}/${version}/${library_name}-${version}-")
    elseif(${library_name} STREQUAL "driver" OR
           ${library_name} STREQUAL "cpp")
        set(BASE_URL "${base_url_string}/${version}/REVLib-${library_name}-${version}-")
    else()
        set(BASE_URL "${base_url_string}/${library_name}/${library_name}-cpp/${version}/${library_name}-cpp-${version}-")
    endif()

    set(HEADER_URL "${BASE_URL}headers.zip" PARENT_SCOPE)
    set(LIB_URL "${BASE_URL}${OS_STRING}${ARCH_STRING}${STATIC_STRING}${BUILD_TYPE_STRING}.zip" PARENT_SCOPE)

    if("${STATIC_STRING}" STREQUAL "")
        set(LINK_TYPE_STRING "shared")
    else()
        set(LINK_TYPE_STRING "static")
    endif()

    if(NOT (${library_name} STREQUAL "apriltaglib" OR 
        ${library_name} STREQUAL "chipobject" OR 
        ${library_name} STREQUAL "netcomm" OR 
        ${library_name} STREQUAL "runtime" OR 
        ${library_name} STREQUAL "visa" OR
        ${library_name} STREQUAL "navx-frc-cpp" OR
        ${library_name} STREQUAL "PathplannerLib-cpp"
    ))
        cmake_print_variables(library_name)
        set(PATH_SUFFIX "${OS_STRING}/${ARCH_STRING}/${LINK_TYPE_STRING}" PARENT_SCOPE)
    else()
        set(PATH_SUFFIX "${ARCH_STRING}/${LINK_TYPE_STRING}" PARENT_SCOPE)
    endif()
endfunction()