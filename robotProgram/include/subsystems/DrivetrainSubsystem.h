#pragma once

#include "ctre-swerve/SwerveDrivebase.h"
#include "frc2/command/SubsystemBase.h"

class DrivetrainSubsystem : public SwerveDrivebase, public frc2::SubsystemBase
{
public:
    DrivetrainSubsystem();
    virtual ~DrivetrainSubsystem();

    frc2::CommandPtr ApplyRequest(std::function<std::unique_ptr<RequestTypes::SwerveRequest>()> requestSupplier);

    void Periodic() override;
    void SimulationPeriodic() override;
    void SeedFieldRelative(frc::Pose2d location) override;

private:
};