// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/simulation/DCMotorSim.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <wpi/interpolating_map.h>

#include <functional>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  frc2::CommandPtr GoToSpeedCmd(std::function<double()> speed);
  frc2::CommandPtr GoToVelocityCmd(
      std::function<units::radians_per_second_t()> speed,
      std::function<bool()> dunk);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);
  frc2::CommandPtr GoToSpeedBasedOnGoal(
      std::function<units::meter_t()> distanceToGoal);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void Set(double speed);

  void GoToVelocity(units::radians_per_second_t speed, bool dunk);
  units::radians_per_second_t GetLeftShooterCurrentVelocity();
  units::radians_per_second_t GetRightShooterCurrentVelocity();

  units::radians_per_second_t GetSetpoint() const {
    return currentVelocitySetpoint;
  }

 private:
  void ConfigureMotors();
  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::shooter::ShooterGains newGains);
  constants::shooter::ShooterGains GetGains();

  void SimulationPeriodic() override;

  bool IsShooterUpToSpeed();

  units::radians_per_second_t ConvertMotorVelToShooterVel(
      units::radians_per_second_t vel);
  units::radians_per_second_t ConvertShooterVelToMotorVel(
      units::radians_per_second_t vel);

  ctre::phoenix6::hardware::TalonFX shooterLeftMotor{
      constants::shooter::LEFT_SHOOTER_CAN_ID};
  ctre::phoenix6::hardware::TalonFX shooterRightMotor{
      constants::shooter::RIGHT_SHOOTER_CAN_ID};

  ctre::phoenix6::controls::VelocityVoltage velocityControl{0_rad_per_s};
  ctre::phoenix6::controls::VoltageOut voltageController{0_V};
  ctre::phoenix6::controls::DutyCycleOut dutyController{0};

  ctre::phoenix6::StatusSignal<units::volt_t> leftShooterVoltageSignal{
      shooterLeftMotor.GetMotorVoltage()};
  ctre::phoenix6::StatusSignal<units::turn_t> leftShooterPosSignal{
      shooterLeftMotor.GetPosition()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> leftShooterVelSignal{
      shooterLeftMotor.GetVelocity()};
  ctre::phoenix6::StatusSignal<units::turns_per_second_t> rightShooterVelSignal{
      shooterRightMotor.GetVelocity()};

  units::radians_per_second_t currentVelocitySetpoint{0};
  units::volt_t currentLeftVoltage{0};
  units::radian_t currentLeftPosition{0};
  units::radians_per_second_t currentLeftVelocity{0};
  units::radians_per_second_t currentRightVelocity{0};

  constants::shooter::ShooterGains currentGains = constants::shooter::GAINS;

  frc::DCMotor shooterGearbox{frc::DCMotor::Falcon500(1)};
  frc::sim::DCMotorSim leftShooterSim{shooterGearbox,
                                      constants::shooter::SHOOTER_RATIO,
                                      constants::shooter::SHOOTER_MOI};

  frc::sim::DCMotorSim rightShooterSim{shooterGearbox,
                                       constants::shooter::SHOOTER_RATIO,
                                       constants::shooter::SHOOTER_MOI};

  ctre::phoenix6::sim::TalonFXSimState& leftShooterSimState{
      shooterLeftMotor.GetSimState()};

  ctre::phoenix6::sim::TalonFXSimState& rightShooterSimState{
      shooterRightMotor.GetSimState()};

  frc2::sysid::SysIdRoutine sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) {
            shooterLeftMotor.SetVoltage(voltsToSend);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("left-shooter")
                .voltage(currentLeftVoltage)
                .position(currentLeftPosition.convert<units::turns>())
                .velocity(GetLeftShooterCurrentVelocity()
                              .convert<units::turns_per_second>());
          },
          this},
  };

  wpi::interpolating_map<units::meter_t, units::radians_per_second_t>
      lookupTable;
};
