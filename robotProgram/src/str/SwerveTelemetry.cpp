// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveTelemetry.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

SwerveTelemetry::SwerveTelemetry(units::meters_per_second_t maxSpeed)
  : maximumSpeed(maxSpeed)
{
}

void SwerveTelemetry::Telemeterize(SwerveDriveState state)
{
  frc::Pose2d pose = state.pose;
  fieldTypePub.Set("Field2d");
  double fieldArr[]
    = {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
  fieldPub.Set(fieldArr);

  units::second_t currentTime
    = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  units::second_t diffTime = currentTime - lastTime;
  lastTime = currentTime;

  frc::Translation2d distanceDiff = (pose - lastPose).Translation();
  lastPose = pose;

  frc::Translation2d velocities = distanceDiff / diffTime.value();

  speed.Set(velocities.Norm().value());
  velocityX.Set(velocities.X().value());
  velocityY.Set(velocities.Y().value());
  odomPeriod.Set(1.0 / state.odometryPeriod.value());

  std::array<double, 8> advantageScopeSwerveView{
    state.moduleStates[0].angle.Degrees().value(),
    state.moduleStates[0].speed.value(),
    state.moduleStates[1].angle.Degrees().value(),
    state.moduleStates[1].speed.value(),
    state.moduleStates[2].angle.Degrees().value(),
    state.moduleStates[2].speed.value(),
    state.moduleStates[3].angle.Degrees().value(),
    state.moduleStates[3].speed.value()};
  swerveStatePub.Set(advantageScopeSwerveView);
}
