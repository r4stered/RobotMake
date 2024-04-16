set_target_properties(robotProgram PROPERTIES FOLDER "str")

target_sources(
    robotProgram
    PRIVATE
        DataUtils.cpp
        LedSection.cpp
        LedStrip.cpp
        MockToF.cpp
        SwerveDrive.cpp
        SwerveDriveSim.cpp
        SwerveModule.cpp
)