// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <wpi/json.h>
#include <wpi/sendable/SendableBuilder.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

struct CharData {
  units::volt_t motorVoltage{0_V};
  units::radian_t motorAngle{0_rad};
  units::radians_per_second_t motorAngleVel{0_rad_per_s};
};

struct ModuleGains {
  double kA{0};
  double kV{0};
  double kS{0};
  double kP{0};
  double kI{0};
  double kD{0};
};

class SwerveModule : public wpi::Sendable {
public:
  SwerveModule(int driveMotorId, int steerMotorId, int steerEncId,
    double steerEncOffset, bool invertDrive, bool invertSteer);
  void GoToState(frc::SwerveModuleState state, bool openLoop);

  frc::SwerveModulePosition GetPosition(bool refresh);
  frc::SwerveModulePosition GetCachedPosition();
  frc::SwerveModuleState GetCurrentState();
  void ResetPosition();
  std::array<ctre::phoenix6::BaseStatusSignal*, 4> GetSignals();

  void SetSteerMotorVolts(units::volt_t voltage);
  CharData GetCharData();

  ctre::phoenix6::hardware::TalonFX driveMotor;
  ctre::phoenix6::hardware::TalonFX steerMotor;
  ctre::phoenix6::hardware::CANcoder steerEnc;

private:
  void ConfigureDriveMotor(bool invertDrive);
  void ConfigureSteerEncoder(double encOffset);
  void ConfigureSteerMotor(bool invertSteer);

  frc::SwerveModulePosition internalState;
  ModuleGains currentSteeringGains;
  ModuleGains currentDrivingGains;

  void SetSteeringGains(const ModuleGains& newGains);
  ModuleGains GetSteeringGains();

  void SetDrivingGains(const ModuleGains& newGains);
  ModuleGains GetDrivingGains();

  ctre::phoenix6::StatusSignal<units::turn_t> steerAngleSig
    = steerMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> steerAngleVelSig
    = steerMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::turn_t> drivePositionSig
    = driveMotor.GetPosition();
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> driveVelocitySig
    = driveMotor.GetVelocity();
  ctre::phoenix6::StatusSignal<units::volt_t> steerVoltageSig
    = steerMotor.GetMotorVoltage();

  ctre::phoenix6::controls::MotionMagicVoltage angleSetter{0_rad};
  ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocityTorqueSetter{
    0_rad_per_s};
  ctre::phoenix6::controls::VoltageOut voltageOpenLoopSetter{0_V};
  ctre::phoenix6::controls::VoltageOut identifySteerSetter{0_V};

  void InitSendable(wpi::SendableBuilder& builder) override;
};
