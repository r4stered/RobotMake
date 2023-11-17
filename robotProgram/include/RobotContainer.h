// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "Autos.h"
#include "subsystems/DrivebaseSubsystem.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  DrivebaseSubsystem& GetDriveSub();

private:
  void ConfigureBindings();
  frc::SendableChooser<std::function<frc2::CommandPtr()>> autoChooser;
  frc2::CommandXboxController driverController{0};
  DrivebaseSubsystem drivebaseSub;
  RequestTypes::FieldCentric drive;
  RequestTypes::FieldCentricFacingAngle driveAtAngle;
  frc2::CommandPtr charModulesCmd = drivebaseSub.CharacterizeSteerMotors(
    [this] { return driverController.GetStartButtonPressed(); });
  Autos autos{&drivebaseSub};
};
