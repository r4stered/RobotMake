// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/DCMotorSim.h>
#include <str/SwerveModule.h>

#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

#include "Constants.h"

struct SimSwerveModule {
  frc::sim::DCMotorSim steerMotor{frc::DCMotor::Falcon500FOC(1),
    constants::drivebase::physical::STEER_GEARING,
    units::moment_of_inertia::kilogram_square_meter_t{0.00001}};
  frc::sim::DCMotorSim driveMotor{frc::DCMotor::Falcon500FOC(1),
    constants::drivebase::physical::DRIVE_GEARING,
    units::moment_of_inertia::kilogram_square_meter_t{0.001}};
};

class SimSwerveDrivetrain {
public:
  SimSwerveDrivetrain(frc::SwerveDriveKinematics<4> kine,
    ctre::phoenix6::hardware::Pigeon2& pigeon);
  void Update(units::second_t deltaTime, units::volt_t supplyVoltage,
    std::array<SwerveModule, 4>& modules);

private:
  frc::SwerveDriveKinematics<4> kinematics;
  ctre::phoenix6::sim::Pigeon2SimState pigeonSim;
  std::array<SimSwerveModule, 4> modules{};
  std::array<frc::SwerveModulePosition, 4> lastPositions{};
  frc::Rotation2d lastAngle;
};
