set_target_properties(robotProgram PROPERTIES FOLDER "subsystems")

target_sources(
    robotProgram
    PRIVATE
        ClimberSubsystem.cpp
        DunkerSubsystem.cpp
        IntakeSubsystem.cpp
        LedSubsystem.cpp
        ShooterSubsystem.cpp
        DrivebaseSubsystem.cpp
)