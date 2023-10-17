// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc/simulation/DCMotorSim.h>
#include "Constants.h"
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <frc/kinematics/SwerveModulePosition.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre-swerve/SwerveModule.h>

struct SimSwerveModule
{
  frc::sim::DCMotorSim steerMotor{frc::DCMotor::Falcon500(1), constants::drivebase::physical::STEER_GEARING, units::moment_of_inertia::kilogram_square_meter_t{0.001}};
  frc::sim::DCMotorSim driveMotor{frc::DCMotor::Falcon500(1), constants::drivebase::physical::DRIVE_GEARING, units::moment_of_inertia::kilogram_square_meter_t{0.001}};
};

class SimSwerveDrivetrain
{
public:
  SimSwerveDrivetrain(ctre::phoenix6::hardware::Pigeon2 &pigeon);
  void Update(units::second_t deltaTime, units::volt_t supplyVoltage, std::array<SwerveModule, 4> &modules);

private:
  ctre::phoenix6::sim::Pigeon2SimState pigeonSim;
  std::array<SimSwerveModule, 4> modules{};
  std::array<frc::SwerveModulePosition, 4> lastPositions{};
  frc::Rotation2d lastAngle{};
};
