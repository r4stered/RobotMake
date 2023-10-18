// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : drive(RequestTypes::FieldCentric().withIsOpenLoop(true).withDeadband(constants::drivebase::physical::MAX_DRIVE_SPEED * 0.1).withRotationalDeadband(3.14_rad_per_s * 0.1))
{
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
  drivetrainSub.SetDefaultCommand(
      drivetrainSub.ApplyRequest(
          [this]
          {
            return std::make_unique<RequestTypes::FieldCentric>(drive.withVelocityX(-driverController.GetLeftY() * constants::drivebase::physical::MAX_DRIVE_SPEED).withVelocityY(-driverController.GetLeftX() * constants::drivebase::physical::MAX_DRIVE_SPEED).withRotationalRate(-driverController.GetRightX() * 3.14_rad_per_s));
          }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
  return frc2::cmd::Print("No autonomous command configured");
}
