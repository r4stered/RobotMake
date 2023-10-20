#pragma once

#include "ctre-swerve/SwerveDrivebase.h"
#include "frc2/command/SubsystemBase.h"
#include <frc/smartdashboard/Field2d.h>

class DrivetrainSubsystem : public frc2::SubsystemBase, public SwerveDrivebase
{
public:
    DrivetrainSubsystem();
    virtual ~DrivetrainSubsystem();

    frc2::CommandPtr ApplyRequest(std::function<std::unique_ptr<RequestTypes::SwerveRequest>()> requestSupplier);

    void Periodic() override;
    void SimulationPeriodic() override;
    void SeedFieldRelative(frc::Pose2d location) override;

private:
    frc::Field2d field{};
};