// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Autos.h"

#include <frc2/command/PrintCommand.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

Autos::Autos(DrivebaseSubsystem* driveSub)
  : m_driveSub(driveSub)
{
  pathplanner::NamedCommands::registerCommand(
    "Print", frc2::PrintCommand("Test Command Print").ToPtr());
}

std::function<frc2::CommandPtr()> Autos::DoNothing()
{
  return [] {
    return frc2::cmd::Print(
      "WARNING: You probably selected the wrong auto mode!");
  };
}

std::function<frc2::CommandPtr()> Autos::GetTestAuto()
{
  return [] { return pathplanner::PathPlannerAuto{"TestAuto"}.ToPtr(); };
}
