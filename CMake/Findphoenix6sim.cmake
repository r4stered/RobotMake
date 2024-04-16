include(${CMAKE_CURRENT_SOURCE_DIR}/CMake/FindWpiPackage.cmake)

set(CTRE_VERSION "24.2.0")

FindWpiPackage(tools-sim "CTRE_PhoenixTools_Sim" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(wpiapi-cpp-sim "CTRE_Phoenix6_WPISim" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simtalonsrx "CTRE_SimTalonSRX" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simtalonfx "CTRE_SimTalonFX" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simvictorspx "CTRE_SimVictorSPX" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simpigeonimu "CTRE_SimPigeonIMU" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simcancoder "CTRE_SimCANCoder" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simprotalonfx "CTRE_SimProTalonFX" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simprocancoder "CTRE_SimProCANcoder" "${CTRE_VERSION}" "" NO NO GetCtreUrl)
FindWpiPackage(simpropigeon2 "CTRE_SimProPigeon2" "${CTRE_VERSION}" "" NO NO GetCtreUrl)

add_library(phoenix6sim INTERFACE)
target_link_libraries(phoenix6sim INTERFACE 
    tools-sim 
    wpiapi-cpp-sim 
    simtalonsrx
    simtalonfx
    simvictorspx
    simpigeonimu
    simcancoder
    simprotalonfx
    simprocancoder
    simpropigeon2
)