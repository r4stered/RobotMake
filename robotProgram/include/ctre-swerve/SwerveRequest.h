// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include "ctre-swerve/SwerveModule.h"

namespace RequestTypes
{
  struct SwerveControlRequestParameters
  {
    frc::SwerveDriveKinematics<4> &kinematics;
    frc::Pose2d currentPose;
    double timestamp;
    std::array<frc::Translation2d, 4> swervePositions;
  };

  class SwerveRequest
  {
  public:
    virtual ~SwerveRequest(){};
    virtual ctre::phoenix::StatusCode Apply(SwerveControlRequestParameters parameters, std::array<SwerveModule, 4> &modules) = 0;
  };

  class Idle : public SwerveRequest
  {
  public:
    bool isOpenLoop = true;
    ctre::phoenix::StatusCode Apply(SwerveControlRequestParameters parameters, std::array<SwerveModule, 4> &modules) override
    {
      fmt::print("{}\n", parameters);
      fmt::print("{}\n", modules);
      return ctre::phoenix::StatusCode::OK;
    };
  };
}
