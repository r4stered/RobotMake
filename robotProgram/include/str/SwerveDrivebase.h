// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>

#include <functional>
#include <memory>
#include <shared_mutex>
#include <thread>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"
#include "str/SimSwerveDrivetrain.h"
#include "str/SwerveModule.h"
#include "str/SwerveRequest.h"

struct SwerveDriveState {
  int successfulDaqs{0};
  int failedDaqs{0};
  frc::Pose2d pose{};
  std::array<frc::SwerveModuleState, 4> moduleStates;
  units::second_t odometryPeriod{0};
};

class SwerveDrivebase {
public:
  explicit SwerveDrivebase(std::function<void(SwerveDriveState)> telemFunc);
  virtual ~SwerveDrivebase();
  void SetControl(std::unique_ptr<RequestTypes::SwerveRequest> request);
  void TareEverything();
  void SeedFieldRelative();
  virtual void SeedFieldRelative(frc::Pose2d location);
  SwerveDriveState GetState();
  void AddVisionMeasurement(frc::Pose2d visionRobotPose,
    units::second_t timestamp, wpi::array<double, 3> visionMeasurementStdDevs);
  void AddVisionMeasurement(
    frc::Pose2d visionRobotPose, units::second_t timestamp);
  void SetVisionMeasurementStdDevs(
    wpi::array<double, 3> visionMeasurementStdDevs);
  void UpdateSimState(units::second_t dt, units::volt_t supplyVoltage);
  bool IsOdometryValid();

protected:
  void UpdateOdometry();

  ctre::phoenix6::hardware::Pigeon2 imu{constants::drivebase::can::IMU, "*"};
  std::array<SwerveModule, 4> modules{
    SwerveModule{constants::drivebase::can::FL_DRIVE,
      constants::drivebase::can::FL_STEER, constants::drivebase::can::FL_ENC,
      constants::drivebase::physical::FL_ENC_OFFSET, false, false},
    SwerveModule{constants::drivebase::can::FR_DRIVE,
      constants::drivebase::can::FR_STEER, constants::drivebase::can::FR_ENC,
      constants::drivebase::physical::FR_ENC_OFFSET, false, false},
    SwerveModule{constants::drivebase::can::BL_DRIVE,
      constants::drivebase::can::BL_STEER, constants::drivebase::can::BL_ENC,
      constants::drivebase::physical::BL_ENC_OFFSET, false, false},
    SwerveModule{constants::drivebase::can::BR_DRIVE,
      constants::drivebase::can::BR_STEER, constants::drivebase::can::BR_ENC,
      constants::drivebase::physical::BR_ENC_OFFSET, false, false}};

  std::array<frc::SwerveModulePosition, 4> modulePostions{
    modules[0].GetPosition(true),
    modules[1].GetPosition(true),
    modules[2].GetPosition(true),
    modules[3].GetPosition(true),
  };

  std::array<frc::Translation2d, 4> moduleLocations{
    frc::Translation2d{constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      constants::drivebase::physical::WHEELBASE_WIDTH / 2},
    frc::Translation2d{constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      -constants::drivebase::physical::WHEELBASE_WIDTH / 2},
    frc::Translation2d{-constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      constants::drivebase::physical::WHEELBASE_WIDTH / 2},
    frc::Translation2d{-constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      -constants::drivebase::physical::WHEELBASE_WIDTH / 2},
  };

  frc::SwerveDriveKinematics<4> kinematics{moduleLocations};

  frc::SwerveDrivePoseEstimator<4> odometry{
    kinematics, frc::Rotation2d{}, modulePostions, frc::Pose2d{}};
  frc::Rotation2d fieldRelativeOffset{};

  std::thread odometryThread{&SwerveDrivebase::UpdateOdometry, this};
  std::shared_mutex lock;

  SwerveDriveState cachedState{};

  SimSwerveDrivetrain simDrivetrain{kinematics, imu};

  std::unique_ptr<RequestTypes::SwerveRequest> requestToApply
    = std::make_unique<RequestTypes::Idle>();
  RequestTypes::SwerveControlRequestParameters requestParameters{kinematics,
    frc::Pose2d{}, 0.0_s,
    {frc::Translation2d{}, frc::Translation2d{}, frc::Translation2d{},
      frc::Translation2d{}}};

  std::function<void(SwerveDriveState)> telemetryFunction;
  bool validOdom{false};
};
