// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SimSwerveDrivetrain.h"

#include <wpi/array.h>

#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

SimSwerveDrivetrain::SimSwerveDrivetrain(
  frc::SwerveDriveKinematics<4> kine, ctre::phoenix6::hardware::Pigeon2& pigeon)
  : kinematics(kine)
  , pigeonSim(pigeon)
{
}

void SimSwerveDrivetrain::Update(units::second_t deltaTime,
  units::volt_t supplyVoltage, std::array<SwerveModule, 4>& modulesToApply)
{
  wpi::array<frc::SwerveModulePosition, 4> positions{
    frc::SwerveModulePosition{}, frc::SwerveModulePosition{},
    frc::SwerveModulePosition{}, frc::SwerveModulePosition{}};
  for (int i = 0; i < 4; i++) {
    ctre::phoenix6::sim::TalonFXSimState& steerMotor{
      modulesToApply[i].steerMotor.GetSimState()};
    ctre::phoenix6::sim::TalonFXSimState& driveMotor{
      modulesToApply[i].driveMotor.GetSimState()};
    ctre::phoenix6::sim::CANcoderSimState& encoder{
      modulesToApply[i].steerEnc.GetSimState()};

    modules[i].steerMotor.SetInputVoltage(steerMotor.GetMotorVoltage());
    modules[i].driveMotor.SetInputVoltage(driveMotor.GetMotorVoltage());

    modules[i].steerMotor.Update(deltaTime);
    modules[i].driveMotor.Update(deltaTime);

    steerMotor.SetRawRotorPosition(modules[i].steerMotor.GetAngularPosition()
      * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetRotorVelocity(modules[i].steerMotor.GetAngularVelocity()
      * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetSupplyVoltage(supplyVoltage);

    encoder.SetRawPosition(modules[i].steerMotor.GetAngularPosition());
    encoder.SetVelocity(modules[i].steerMotor.GetAngularVelocity());
    encoder.SetSupplyVoltage(supplyVoltage);

    driveMotor.SetRawRotorPosition(modules[i].driveMotor.GetAngularPosition()
      * constants::drivebase::physical::DRIVE_GEARING);
    driveMotor.SetRotorVelocity(modules[i].driveMotor.GetAngularVelocity()
      * constants::drivebase::physical::DRIVE_GEARING);
    driveMotor.SetSupplyVoltage(supplyVoltage);

    frc::SwerveModulePosition currentPosition
      = modulesToApply[i].GetCachedPosition();
    positions[i] = frc::SwerveModulePosition{
      currentPosition.distance - lastPositions[i].distance,
      currentPosition.angle};
    lastPositions[i].distance = currentPosition.distance;
  }

  frc::Twist2d change = kinematics.ToTwist2d(positions);
  lastAngle = lastAngle + frc::Rotation2d{change.dtheta};
  pigeonSim.SetRawYaw(lastAngle.Radians());
}
