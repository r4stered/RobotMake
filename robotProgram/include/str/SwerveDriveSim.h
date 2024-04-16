// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include "Constants.h"
#include "frc/simulation/DCMotorSim.h"
#include "frc/system/plant/DCMotor.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/velocity.h"

namespace str {

struct SwerveModuleSim {
  frc::sim::DCMotorSim steerMotor{frc::DCMotor::Falcon500(1),
                                  constants::swerve::physical::STEER_GEARING,
                                  0.025_kg_sq_m};
  frc::sim::DCMotorSim driveMotor{frc::DCMotor::Falcon500(1),
                                  constants::swerve::physical::DRIVE_GEARING,
                                  0.025_kg_sq_m};
};

struct SimState {
  units::meter_t drivePos{};
  units::meters_per_second_t driveVel{};
  units::radian_t steerPos{};
  units::radians_per_second_t steerVel{};
};

class SwerveDriveSim {
 public:
  void SetDriveInputs(const std::array<units::volt_t, 4>& inputs);
  void SetSteerInputs(const std::array<units::volt_t, 4>& inputs);
  void Update(units::second_t dt);
  units::ampere_t GetDriveCurrentDraw() const;
  units::ampere_t GetSteerCurrentDraw() const;
  std::array<SimState, 4> GetState() const;

 private:
  std::array<SwerveModuleSim, 4> simModules{};
};
}  // namespace str
