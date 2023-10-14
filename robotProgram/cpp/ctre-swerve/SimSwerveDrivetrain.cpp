// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ctre-swerve/SimSwerveDrivetrain.h"
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <ctre/phoenix6/sim/Pigeon2SimState.hpp>

SimSwerveDrivetrain::SimSwerveDrivetrain(std::array<frc::Translation2d, 4> wheelLocations, ctre::phoenix6::hardware::Pigeon2 &pigeon) : kinematics{wheelLocations},
                                                                                                                                        pigeonSim{pigeon}
{
}

void SimSwerveDrivetrain::Update(units::second_t deltaTime, units::volt_t supplyVoltage, std::array<SwerveModule, 4> &modulesToApply)
{
  std::array<frc::SwerveModulePosition, 4> positions{};
  for (unsigned int i = 0; i < 4; i++)
  {
    ctre::phoenix6::sim::TalonFXSimState &steerMotor{modulesToApply[i].steerMotor.GetSimState()};
    ctre::phoenix6::sim::TalonFXSimState &driveMotor{modulesToApply[i].driveMotor.GetSimState()};
    ctre::phoenix6::sim::CANcoderSimState &encoder{modulesToApply[i].steerEnc.GetSimState()};

    modules[i].steerMotor.SetInputVoltage(steerMotor.GetMotorVoltage());
    modules[i].driveMotor.SetInputVoltage(driveMotor.GetMotorVoltage());

    modules[i].steerMotor.Update(deltaTime);
    modules[i].driveMotor.Update(deltaTime);

    steerMotor.SetRawRotorPosition(modules[i].steerMotor.GetAngularPosition() * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetRotorVelocity(modules[i].driveMotor.GetAngularVelocity() / 60.0 * constants::drivebase::physical::STEER_GEARING);
    steerMotor.SetSupplyVoltage(supplyVoltage);

    encoder.SetRawPosition(modules[i].steerMotor.GetAngularPosition());
    encoder.SetVelocity(modules[i].steerMotor.GetAngularVelocity() / 60.0);
    encoder.SetSupplyVoltage(supplyVoltage);

    driveMotor.SetRawRotorPosition(modules[i].driveMotor.GetAngularPosition() * constants::drivebase::physical::DRIVE_GEARING);
    driveMotor.SetRotorVelocity(modules[i].driveMotor.GetAngularVelocity() / 60.0 * constants::drivebase::physical::DRIVE_GEARING);
    driveMotor.SetSupplyVoltage(supplyVoltage);

    frc::SwerveModulePosition currentPosition = modulesToApply[i].GetCachedPosition();
    positions[i] = frc::SwerveModulePosition{currentPosition.distance - lastPositions[i].distance, currentPosition.angle};
    lastPositions[i].distance = currentPosition.distance;
  }

  // frc::Twist2d change = kinematics.ToTwist2d(&positions);
  // lastAngle = lastAngle + frc::Rotation2d{change.dtheta};
  pigeonSim.SetRawYaw(lastAngle.Radians());
}
