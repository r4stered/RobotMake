// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ctre-swerve/SwerveDrivebase.h"
#include <frc/filter/LinearFilter.h>
#include <ctre/phoenix6/Utils.hpp>
#include <fmt/format.h>

SwerveDrivebase::SwerveDrivebase()
{
}

SwerveDrivebase::~SwerveDrivebase()
{
  if (odometryThread.joinable())
  {
    odometryThread.join();
  }
}

void SwerveDrivebase::UpdateOdometry()
{

  std::array<ctre::phoenix6::BaseStatusSignal *, 18> allSignals;
  int successfulDaqs = 0;
  int failedDaqs = 0;
  frc::LinearFilter<double> lowpass = frc::LinearFilter<double>::MovingAverage(50);
  double lastTime = 0;
  double currentTime = 0;
  double averageLoopTime = 0;

  for (int i = 0; i < 4; i++)
  {
    std::array<ctre::phoenix6::BaseStatusSignal *, 4> signals = modules[i].GetSignals();
    allSignals[(i * 4) + 0] = signals[0];
    allSignals[(i * 4) + 1] = signals[1];
    allSignals[(i * 4) + 2] = signals[2];
    allSignals[(i * 4) + 3] = signals[3];
  }
  allSignals[allSignals.size() - 2] = &imu.GetYaw();
  allSignals[allSignals.size() - 1] = &imu.GetAngularVelocityZ();

  for (ctre::phoenix6::BaseStatusSignal *sig : allSignals)
  {
    sig->SetUpdateFrequency(250_Hz);
  }

  fmt::print("About to go infinite >:)\n");

  // runs in seperate thread so we chillin'
  while (true)
  {
    ctre::phoenix::StatusCode status;
    status = ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz, allSignals);

    std::unique_lock<std::shared_mutex> writeLock(lock);

    lastTime = currentTime;
    currentTime = ctre::phoenix6::GetCurrentTimeSeconds();
    averageLoopTime = lowpass.Calculate(currentTime - lastTime);

    if (status.IsOK())
    {
      successfulDaqs++;
    }
    else
    {
      failedDaqs++;
    }

    for (int i = 0; i < 4; i++)
    {
      modulePostions[i] = modules[i].GetPosition(false);
    }

    units::radian_t imuYaw = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(imu.GetYaw(), imu.GetAngularVelocityZ());
    odometry.Update(frc::Rotation2d{imuYaw}, modulePostions);

    requestParameters.currentPose = odometry.GetEstimatedPosition().RelativeTo(frc::Pose2d{0_m, 0_m, fieldRelativeOffset});
    requestParameters.kinematics = kinematics;
    requestParameters.swervePositions = moduleLocations;
    requestParameters.timestamp = currentTime;

    requestToApply->Apply(requestParameters, modules);

    cachedState.failedDaqs = failedDaqs;
    cachedState.successfulDaqs = successfulDaqs;
    cachedState.moduleStates = {modules[0].GetCurrentState(), modules[1].GetCurrentState(), modules[2].GetCurrentState(), modules[3].GetCurrentState()};
    cachedState.pose = odometry.GetEstimatedPosition();
    cachedState.odometryPeriod = averageLoopTime;
  }
}
