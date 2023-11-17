// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include <functional>
#include <memory>

#include "str/SwerveDrivebase.h"
#include "str/SwerveTelemetry.h"

class DrivebaseSubsystem : public frc2::SubsystemBase, public SwerveDrivebase {
public:
  DrivebaseSubsystem();

  frc2::CommandPtr ApplyRequest(
    std::function<std::unique_ptr<RequestTypes::SwerveRequest>()>
      requestSupplier);

  frc2::CommandPtr CharacterizeSteerMotors(
    std::function<bool()> nextStepButton);

  void Periodic() override;
  void SimulationPeriodic() override;
  void SeedFieldRelative(frc::Pose2d location) override;

private:
  SwerveTelemetry telem{constants::drivebase::physical::MAX_DRIVE_SPEED};
  void SetupAutoBuilder();
  RequestTypes::ApplyChassisSpeeds autoRequest{};

  // This is for characterization
  wpi::json flModuleData{};
  wpi::json frModuleData{};
  wpi::json blModuleData{};
  wpi::json brModuleData{};

  units::volt_t quasistaticVolts = 0_V;
  static constexpr auto quasistaticStep{0.25_V / 1_s};
  units::volt_t dynamicStepVolts = 7_V;
};
