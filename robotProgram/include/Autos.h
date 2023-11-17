// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <functional>

#include "subsystems/DrivebaseSubsystem.h"

class Autos {
public:
  explicit Autos(DrivebaseSubsystem* driveSub);
  std::function<frc2::CommandPtr()> GetTestAuto();
  std::function<frc2::CommandPtr()> DoNothing();

private:
  DrivebaseSubsystem* m_driveSub;
};
