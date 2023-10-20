// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <subsystems/DrivetrainSubsystem.h>
#include "str/DrivetrainTelemetry.h"

class RobotContainer
{
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  void ConfigureBindings();
  frc2::CommandXboxController driverController{0};
  //DrivetrainTelemetry driveTelem{constants::drivebase::physical::MAX_DRIVE_SPEED};
  DrivetrainSubsystem drivetrainSub;
  RequestTypes::FieldCentric drive;
};
