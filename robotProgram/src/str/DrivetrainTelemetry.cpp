// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "str/DrivetrainTelemetry.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/DoubleTopic.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

DrivetrainTelemetry::DrivetrainTelemetry(units::meters_per_second_t maxSpeed) : maximumSpeed(maxSpeed) 
{

}

void DrivetrainTelemetry::Telemeterize(/*SwerveDriveState state*/) {
  frc::Pose2d pose{}; //= state.pose;
  fieldTypePub.Set("Field2d");
  double fieldArr[] = {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
  fieldPub.Set(fieldArr);
  
  units::second_t currentTime = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  units::second_t diffTime = currentTime - lastTime;
  lastTime = currentTime;

  frc::Translation2d distanceDiff = (pose - lastPose).Translation();
  lastPose = pose;

  frc::Translation2d velocities = distanceDiff / diffTime.value();

  speed.Set(velocities.Norm().value());
  velocityX.Set(velocities.X().value());
  velocityY.Set(velocities.Y().value());
  odomPeriod.Set(1.0 / 1.0/*state.odometryPeriod.value()*/);

  for(size_t i = 0; i < 4; i++) {
    moduleSpeedVis[i]->SetAngle(/*state.moduleStates[i].angle.Degrees()*/0_rad);
    moduleDirections[i]->SetLength(/*state.moduleStates[i].speed*/(2 * maximumSpeed) / (2 * maximumSpeed));
    frc::SmartDashboard::PutData("Module " + std::to_string(i), &moduleVis[i]);
  }

  double logValues[] = {pose.X().value(), pose.Y().value(), pose.Rotation().Degrees().value()};
  frc::DataLogManager::GetLog().AppendDoubleArray(logEntry, logValues, frc::Timer::GetFPGATimestamp().value());
  frc::DataLogManager::GetLog().AppendDouble(odomEntry, /*state.odometryPeriod.value()*/1.0, frc::Timer::GetFPGATimestamp().value());
}