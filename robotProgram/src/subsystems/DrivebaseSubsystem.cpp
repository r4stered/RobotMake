// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DrivebaseSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>

#include <fstream>

DrivebaseSubsystem::DrivebaseSubsystem()
  : SwerveDrivebase(
    [this](SwerveDriveState state) { return telem.Telemeterize(state); })
{
  SetupAutoBuilder();
}

frc2::CommandPtr DrivebaseSubsystem::ApplyRequest(
  std::function<std::unique_ptr<RequestTypes::SwerveRequest>()> requestSupplier)
{
  return frc2::RunCommand{
    [this, requestSupplier] { SetControl(requestSupplier()); }, {this}}
    .ToPtr();
}

// This method will be called once per scheduler run
void DrivebaseSubsystem::Periodic() { }

void DrivebaseSubsystem::SimulationPeriodic()
{
  UpdateSimState(constants::ROBOT_DT, 12_V);
}

void DrivebaseSubsystem::SeedFieldRelative(frc::Pose2d location)
{
  std::unique_lock<std::shared_mutex> writeLock(lock);
  odometry.ResetPosition(
    frc::Rotation2d{imu.GetYaw().GetValue()}, modulePostions, location);
}

void DrivebaseSubsystem::SetupAutoBuilder()
{
  pathplanner::AutoBuilder::configureHolonomic(
    [this] { return GetState().pose; },
    [this](frc::Pose2d resetPose) { SeedFieldRelative(resetPose); },
    [] { return frc::ChassisSpeeds{}; },
    [this](frc::ChassisSpeeds speeds) {
      SetControl(std::make_unique<RequestTypes::ApplyChassisSpeeds>(
        autoRequest.withSpeeds(speeds)));
    },
    pathplanner::HolonomicPathFollowerConfig(
      pathplanner::PIDConstants{constants::drivebase::gains::TRANSLATION_P,
        constants::drivebase::gains::TRANSLATION_I,
        constants::drivebase::gains::TRANSLATION_D},
      pathplanner::PIDConstants{constants::drivebase::gains::TRANSLATION_P,
        constants::drivebase::gains::TRANSLATION_I,
        constants::drivebase::gains::TRANSLATION_D},
      constants::drivebase::physical::MAX_DRIVE_SPEED,
      constants::drivebase::physical::WHEELBASE_LENGTH / 2,
      pathplanner::ReplanningConfig{false}),
    this);
}

frc2::CommandPtr DrivebaseSubsystem::CharacterizeSteerMotors(
  std::function<bool()> nextStepButton)
{
  // clang-format off
  return frc2::cmd::Sequence(
    // SLOW FORWARD
    frc2::cmd::RunOnce([this] {
      fmt::print("Slow Forward Starting...\n");
      flModuleData["slow-forward"] = wpi::json::array();
    }),
    frc2::cmd::RunEnd([this] {
      quasistaticVolts = quasistaticVolts + (quasistaticStep * 20_ms);
      modules[0].SetSteerMotorVolts(quasistaticVolts);

      const auto& flData = modules[0].GetCharData();

      wpi::json dataToAdd = {
        frc::Timer::GetFPGATimestamp().value(),
        flData.motorVoltage.value(),
        flData.motorAngle.value(),
        flData.motorAngleVel.value()
      };

      flModuleData["slow-forward"].push_back(dataToAdd);
    },
    [this] {
      modules[0].SetSteerMotorVolts(0_V);
      quasistaticVolts = 0_V;
    }, {this})
    .Until(nextStepButton),
    frc2::cmd::Wait(1_s),
    // SLOW BACKWARDS
    frc2::cmd::RunOnce([this] {
      fmt::print("Slow Backwards Starting...\n");
      flModuleData["slow-backward"] = wpi::json::array();
    }),
    frc2::cmd::RunEnd([this] {
      quasistaticVolts = quasistaticVolts + (quasistaticStep * 20_ms);
      modules[0].SetSteerMotorVolts(-quasistaticVolts);

      const auto& flData = modules[0].GetCharData();

      wpi::json dataToAdd = {
        frc::Timer::GetFPGATimestamp().value(),
        flData.motorVoltage.value(),
        flData.motorAngle.value(),
        flData.motorAngleVel.value()
      };

      flModuleData["slow-backward"].push_back(dataToAdd);
    },
    [this] {
      modules[0].SetSteerMotorVolts(0_V);
      quasistaticVolts = 0_V;
    }, {this})
    .Until(nextStepButton),
    frc2::cmd::Wait(1_s),
    // FAST FORWARD
    frc2::cmd::RunOnce([this] {
      fmt::print("Fast Forward Starting...\n");
      flModuleData["fast-forward"] = wpi::json::array();
    }),
    frc2::cmd::RunEnd([this] {
      modules[0].SetSteerMotorVolts(7_V);

      const auto& flData = modules[0].GetCharData();

      wpi::json dataToAdd = {
        frc::Timer::GetFPGATimestamp().value(),
        flData.motorVoltage.value(),
        flData.motorAngle.value(),
        flData.motorAngleVel.value()
      };

      flModuleData["fast-forward"].push_back(dataToAdd);
    },
    [this] {
      modules[0].SetSteerMotorVolts(0_V);
    }, {this})
    .Until(nextStepButton),
    frc2::cmd::Wait(1_s),
    // FAST BACKWARDS
    frc2::cmd::RunOnce([this] {
      fmt::print("Fast Backward Starting...\n");
      flModuleData["fast-backward"] = wpi::json::array();
    }),
    frc2::cmd::RunEnd([this] {
      modules[0].SetSteerMotorVolts(-7_V);

      const auto& flData = modules[0].GetCharData();

      wpi::json dataToAdd = {
        frc::Timer::GetFPGATimestamp().value(),
        flData.motorVoltage.value(),
        flData.motorAngle.value(),
        flData.motorAngleVel.value()
      };

      flModuleData["fast-backward"].push_back(dataToAdd);
    },
    [this] {
      modules[0].SetSteerMotorVolts(0_V);
    }, {this})
    .Until(nextStepButton),
    frc2::cmd::Wait(1_s),
    frc2::cmd::RunOnce([this] {
      fmt::print("Done characterizing...\n");
      flModuleData["sysid"] = "true";
      flModuleData["test"] = "Simple";
      flModuleData["units"] = "Radians";
      flModuleData["unitsPerRotation"] = 1.0;
      std::ofstream outFile;
      outFile.open("charData.json");
      outFile << flModuleData.dump() << std::endl;
      outFile.close();
    })
  );
  // clang-format on
}
