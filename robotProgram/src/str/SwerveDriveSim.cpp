// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDriveSim.h"

#include "Constants.h"
#include "str/SwerveModule.h"
#include "units/current.h"

using namespace str;

void SwerveDriveSim::SetDriveInputs(
    const std::array<units::volt_t, 4>& inputs) {
  for (size_t i = 0; i < simModules.size(); i++) {
    simModules[i].driveMotor.SetInputVoltage(inputs[i]);
  }
}

void SwerveDriveSim::SetSteerInputs(
    const std::array<units::volt_t, 4>& inputs) {
  for (size_t i = 0; i < simModules.size(); i++) {
    simModules[i].steerMotor.SetInputVoltage(inputs[i]);
  }
}

void SwerveDriveSim::Update(units::second_t dt) {
  for (size_t i = 0; i < simModules.size(); i++) {
    simModules[i].driveMotor.Update(dt);
    simModules[i].steerMotor.Update(dt);
  }
}

units::ampere_t SwerveDriveSim::GetDriveCurrentDraw() const {
  units::ampere_t total;
  for (size_t i = 0; i < simModules.size(); i++) {
    total = total + simModules[i].driveMotor.GetCurrentDraw();
  }
  return total;
}

units::ampere_t SwerveDriveSim::GetSteerCurrentDraw() const {
  units::ampere_t total;
  for (size_t i = 0; i < simModules.size(); i++) {
    total = total + simModules[i].steerMotor.GetCurrentDraw();
  }
  return total;
}

std::array<SimState, 4> SwerveDriveSim::GetState() const {
  std::array<SimState, 4> state;

  for (size_t i = 0; i < simModules.size(); i++) {
    state[i].drivePos = SwerveModule::ConvertMotorToWheelDistance(
        simModules[i].driveMotor.GetAngularPosition() *
        constants::swerve::physical::DRIVE_GEARING);
    state[i].driveVel = SwerveModule::ConvertMotorSpeedToWheelVelocity(
        simModules[i].driveMotor.GetAngularVelocity() *
        constants::swerve::physical::DRIVE_GEARING);
    state[i].steerPos = simModules[i].steerMotor.GetAngularPosition();
    state[i].steerVel = simModules[i].steerMotor.GetAngularVelocity();
  }

  return state;
}
