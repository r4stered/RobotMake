// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <frc/geometry/Pose2d.h>
#include <ctre/phoenix6/Utils.hpp>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/util/Color8Bit.h>
#include "ctre-swerve/SwerveDrivebase.h"
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>
#include <array>

class DrivetrainTelemetry {
 public:
  DrivetrainTelemetry(units::meters_per_second_t maxSpeed);
  void Telemeterize(/*SwerveDriveState state*/);
 private:
  units::meters_per_second_t maximumSpeed;
  int logEntry{frc::DataLogManager::GetLog().Start("odometry", "double[]")};
  int odomEntry{frc::DataLogManager::GetLog().Start("odom period", "double")};

  nt::NetworkTableInstance inst{nt::NetworkTableInstance::GetDefault()};
  std::shared_ptr<nt::NetworkTable> table{inst.GetTable("Pose")};
  nt::DoubleArrayPublisher fieldPub{table->GetDoubleArrayTopic("robotPose").Publish()};
  nt::StringPublisher fieldTypePub{table->GetStringTopic(".type").Publish()};

  std::shared_ptr<nt::NetworkTable> driveStats{inst.GetTable("Drive")};
  nt::DoublePublisher velocityX{driveStats->GetDoubleTopic("Velocity X").Publish()};
  nt::DoublePublisher velocityY{driveStats->GetDoubleTopic("Velocity Y").Publish()};
  nt::DoublePublisher speed{driveStats->GetDoubleTopic("Speed").Publish()};
  nt::DoublePublisher odomPeriod{driveStats->GetDoubleTopic("Odometry Period").Publish()};

  frc::Pose2d lastPose{};
  units::second_t lastTime{ctre::phoenix6::GetCurrentTimeSeconds()};

  std::array<frc::Mechanism2d, 4> moduleVis{frc::Mechanism2d{1, 1}, frc::Mechanism2d{1, 1}, frc::Mechanism2d{1, 1}, frc::Mechanism2d{1, 1}};
  std::array<frc::MechanismLigament2d*, 4> moduleSpeedVis{
    moduleVis[0].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_rad),
    moduleVis[1].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_rad),
    moduleVis[2].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_rad),
    moduleVis[3].GetRoot("RootSpeed", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_rad)
  };

  std::array<frc::MechanismLigament2d*, 4> moduleDirections{
    moduleVis[0].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_rad, 0, frc::Color::kWhite),
    moduleVis[1].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_rad, 0, frc::Color::kWhite),
    moduleVis[2].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_rad, 0, frc::Color::kWhite),
    moduleVis[3].GetRoot("RootDirection", 0.5, 0.5)->Append<frc::MechanismLigament2d>("Direction", 0.1, 0_rad, 0, frc::Color::kWhite)
  };
};
