// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix/StatusCodes.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/geometry/Pose2d.h>
#include <ctre-swerve/SwerveModule.h>
#include <ctre-swerve/CTREPIDController.h>

namespace RequestTypes
{
  struct SwerveControlRequestParameters
  {
    frc::SwerveDriveKinematics<4> kinematics;
    frc::Pose2d currentPose;
    units::second_t timestamp;
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
      return ctre::phoenix::StatusCode::OK;
    };
  };

  class Brake : public SwerveRequest
  {
  public:
    bool isOpenLoop = true;
    ctre::phoenix::StatusCode Apply(SwerveControlRequestParameters parameters, std::array<SwerveModule, 4> &modules) override
    {
      for (int i = 0; i < modules.size(); i++)
      {
        frc::SwerveModuleState state{0_mps, parameters.swervePositions[i].Angle()};
        modules[i].GoToState(state, isOpenLoop);
      }
      return ctre::phoenix::StatusCode::OK;
    };

    Brake &withIsOpenLoop(bool isOpenLoop)
    {
      this->isOpenLoop = isOpenLoop;
      return *this;
    }
  };

  class FieldCentric : public SwerveRequest
  {
  public:
    units::meters_per_second_t velocityX;
    units::meters_per_second_t velocityY;
    units::radians_per_second_t rotationalRate;
    units::meters_per_second_t deadband = 0_mps;
    units::radians_per_second_t rotationalDeadband = 0_rad_per_s;
    bool isOpenLoop = true;
    ctre::phoenix::StatusCode Apply(SwerveControlRequestParameters parameters, std::array<SwerveModule, 4> &modules) override
    {
      units::meters_per_second_t toApplyX = velocityX;
      units::meters_per_second_t toApplyY = velocityY;
      units::radians_per_second_t toApplyOmega = rotationalRate;
      if (units::math::sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < deadband)
      {
        toApplyX = 0_mps;
        toApplyY = 0_mps;
      }
      if (units::math::abs(toApplyOmega) < rotationalDeadband)
      {
        toApplyOmega = 0_rad_per_s;
      }
      frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega, parameters.currentPose.Rotation());
      auto states = parameters.kinematics.ToSwerveModuleStates(speeds, frc::Translation2d{});
      for (int i = 0; i < modules.size(); i++)
      {
        modules[i].GoToState(states[i], isOpenLoop);
      }
      return ctre::phoenix::StatusCode::OK;
    };

    FieldCentric &withIsOpenLoop(bool isOpenLoop)
    {
      this->isOpenLoop = isOpenLoop;
      return *this;
    }

    FieldCentric &withVelocityX(units::meters_per_second_t velX)
    {
      this->velocityX = velX;
      return *this;
    }

    FieldCentric &withVelocityY(units::meters_per_second_t velY)
    {
      this->velocityY = velY;
      return *this;
    }

    FieldCentric &withRotationalRate(units::radians_per_second_t omega)
    {
      this->rotationalRate = omega;
      return *this;
    }

    FieldCentric &withDeadband(units::meters_per_second_t deadband)
    {
      this->deadband = deadband;
      return *this;
    }

    FieldCentric &withRotationalDeadband(units::radians_per_second_t rotDeadband)
    {
      this->rotationalDeadband = rotDeadband;
      return *this;
    }

  private:
    std::array<frc::SwerveModuleState, 4> lastAppliedState{};
  };

  class FieldCentricFacingAngle : public SwerveRequest
  {
  public:
    units::meters_per_second_t velocityX;
    units::meters_per_second_t velocityY;
    frc::Rotation2d targetDirection;
    units::meters_per_second_t deadband = 0_mps;
    units::radians_per_second_t rotationalDeadband = 0_rad_per_s;
    bool isOpenLoop = true;
    CTREPIDController headingController{0, 0, 0};
    ctre::phoenix::StatusCode Apply(SwerveControlRequestParameters parameters, std::array<SwerveModule, 4> &modules) override
    {
      units::meters_per_second_t toApplyX = velocityX;
      units::meters_per_second_t toApplyY = velocityY;
      units::radians_per_second_t toApplyOmega = units::radians_per_second_t{headingController.Calculate(parameters.currentPose.Rotation().Radians().value(), targetDirection.Radians().value(), parameters.timestamp)};
      if (units::math::sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < deadband)
      {
        toApplyX = 0_mps;
        toApplyY = 0_mps;
      }
      if (units::math::abs(toApplyOmega) < rotationalDeadband)
      {
        toApplyOmega = 0_rad_per_s;
      }
      frc::ChassisSpeeds speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega, parameters.currentPose.Rotation());
      auto states = parameters.kinematics.ToSwerveModuleStates(speeds, frc::Translation2d{});
      for (int i = 0; i < modules.size(); i++)
      {
        modules[i].GoToState(states[i], isOpenLoop);
      }
      return ctre::phoenix::StatusCode::OK;
    };

    FieldCentricFacingAngle &withIsOpenLoop(bool isOpenLoop)
    {
      this->isOpenLoop = isOpenLoop;
      return *this;
    }

    FieldCentricFacingAngle &withVelocityX(units::meters_per_second_t velX)
    {
      this->velocityX = velX;
      return *this;
    }

    FieldCentricFacingAngle &withVelocityY(units::meters_per_second_t velY)
    {
      this->velocityY = velY;
      return *this;
    }

    FieldCentricFacingAngle &withTargetDirection(units::radian_t theta)
    {
      this->targetDirection = frc::Rotation2d{theta};
      return *this;
    }

    FieldCentricFacingAngle &withDeadband(units::meters_per_second_t deadband)
    {
      this->deadband = deadband;
      return *this;
    }

    FieldCentricFacingAngle &withRotationalDeadband(units::radians_per_second_t rotDeadband)
    {
      this->rotationalDeadband = rotDeadband;
      return *this;
    }

  private:
    std::array<frc::SwerveModuleState, 4> lastAppliedState{};
  };
}
