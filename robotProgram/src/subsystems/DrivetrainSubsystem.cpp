#include "subsystems/DrivetrainSubsystem.h"
#include <frc2/command/RunCommand.h>

DrivetrainSubsystem::DrivetrainSubsystem() : SwerveDrivebase()
{
}

DrivetrainSubsystem::~DrivetrainSubsystem()
{
}

frc2::CommandPtr DrivetrainSubsystem::ApplyRequest(std::function<std::unique_ptr<RequestTypes::SwerveRequest>()> requestSupplier)
{
    return frc2::RunCommand{[this, requestSupplier]
                            { SetControl(requestSupplier()); }}
        .ToPtr();
}

void DrivetrainSubsystem::Periodic()
{
}

void DrivetrainSubsystem::SimulationPeriodic()
{
    UpdateSimState(constants::ROBOT_DT, 12_V);
}

void DrivetrainSubsystem::SeedFieldRelative(frc::Pose2d location)
{
    std::unique_lock<std::shared_mutex> writeLock(lock);
    odometry.ResetPosition(frc::Rotation2d{imu.GetYaw().GetValue()}, modulePostions, location);
}