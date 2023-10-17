// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include "Constants.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "ctre-swerve/SwerveModule.h"
#include <thread>
#include <shared_mutex>
#include "ctre-swerve/SwerveRequest.h"

struct SwerveDriveState
{
  int successfulDaqs{0};
  int failedDaqs{0};
  frc::Pose2d pose{};
  std::array<frc::SwerveModuleState, 4> moduleStates;
  double odometryPeriod{0};
};

class SwerveDrivebase
{
public:
  SwerveDrivebase();
  ~SwerveDrivebase();

private:
  void UpdateOdometry();

  ctre::phoenix6::hardware::Pigeon2 imu{constants::drivebase::can::IMU, "*"};
  std::array<SwerveModule, 4> modules{
      SwerveModule{constants::drivebase::can::FL_DRIVE, constants::drivebase::can::FL_STEER, constants::drivebase::can::FL_ENC, constants::drivebase::physical::FL_ENC_OFFSET, false, false},
      SwerveModule{constants::drivebase::can::FR_DRIVE, constants::drivebase::can::FR_STEER, constants::drivebase::can::FR_ENC, constants::drivebase::physical::FR_ENC_OFFSET, false, false},
      SwerveModule{constants::drivebase::can::BL_DRIVE, constants::drivebase::can::BL_STEER, constants::drivebase::can::BL_ENC, constants::drivebase::physical::BL_ENC_OFFSET, false, false},
      SwerveModule{constants::drivebase::can::BR_DRIVE, constants::drivebase::can::BR_STEER, constants::drivebase::can::BR_ENC, constants::drivebase::physical::BR_ENC_OFFSET, false, false}};

  std::array<frc::SwerveModulePosition, 4> modulePostions{
      modules[0].GetPosition(true),
      modules[1].GetPosition(true),
      modules[2].GetPosition(true),
      modules[3].GetPosition(true),
  };

  frc::SwerveDrivePoseEstimator<4> odometry{constants::drivebase::kinematics::kinematics, frc::Rotation2d{}, modulePostions, frc::Pose2d{}};
  frc::Rotation2d fieldRelativeOffset{};

  std::thread odometryThread{&SwerveDrivebase::UpdateOdometry, this};
  std::shared_mutex lock;

  SwerveDriveState cachedState{};

  std::unique_ptr<RequestTypes::SwerveRequest> requestToApply = std::make_unique<RequestTypes::Idle>();
  RequestTypes::SwerveControlRequestParameters requestParameters{constants::drivebase::kinematics::kinematics, frc::Pose2d{}, 0.0, {frc::Translation2d{}, frc::Translation2d{}, frc::Translation2d{}, frc::Translation2d{}}};
};
