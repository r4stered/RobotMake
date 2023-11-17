// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/DataLogManager.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/util/Color8Bit.h>
#include <networktables/NetworkTableInstance.h>
#include <units/velocity.h>
#include <wpi/DataLog.h>

#include <memory>

#include <ctre/phoenix6/Utils.hpp>

#include "str/SwerveDrivebase.h"

class SwerveTelemetry {
public:
  explicit SwerveTelemetry(units::meters_per_second_t maxSpeed);
  void Telemeterize(SwerveDriveState state);

private:
  units::meters_per_second_t maximumSpeed;

  nt::NetworkTableInstance inst{nt::NetworkTableInstance::GetDefault()};
  std::shared_ptr<nt::NetworkTable> table{inst.GetTable("Pose")};
  nt::DoubleArrayPublisher fieldPub{
    table->GetDoubleArrayTopic("robotPose").Publish()};
  nt::StringPublisher fieldTypePub{table->GetStringTopic(".type").Publish()};

  std::shared_ptr<nt::NetworkTable> driveStats{inst.GetTable("Drive")};
  nt::DoublePublisher velocityX{
    driveStats->GetDoubleTopic("Velocity X").Publish()};
  nt::DoublePublisher velocityY{
    driveStats->GetDoubleTopic("Velocity Y").Publish()};
  nt::DoublePublisher speed{driveStats->GetDoubleTopic("Speed").Publish()};
  nt::DoublePublisher odomPeriod{
    driveStats->GetDoubleTopic("Odometry Period").Publish()};
  nt::DoubleArrayPublisher swerveStatePub{
    driveStats->GetDoubleArrayTopic("Swerve Module Actual States").Publish()};

  frc::Pose2d lastPose{};
  units::second_t lastTime{ctre::phoenix6::GetCurrentTimeSeconds()};
};
