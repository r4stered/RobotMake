// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ShooterSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

#include <ctre/phoenix6/SignalLogger.hpp>

ShooterSubsystem::ShooterSubsystem() {
  ConfigureMotors();
  frc::SmartDashboard::PutData("Shooter Telemetry", this);
  lookupTable.insert(0_ft, 1000_rpm);
  lookupTable.insert(10_ft, frc::DCMotor::Falcon500(1).freeSpeed);
}

frc2::CommandPtr ShooterSubsystem::GoToSpeedCmd(std::function<double()> speed) {
  return frc2::cmd::Run([this, speed] { Set(speed()); }, {this})
      .FinallyDo([this] { Set(0); });
}

frc2::CommandPtr ShooterSubsystem::GoToVelocityCmd(
    std::function<units::radians_per_second_t()> speed,
    std::function<bool()> dunk) {
  return frc2::cmd::Run([this, speed, dunk] { GoToVelocity(speed(), dunk()); },
                        {this})
      .Until([this] { return IsShooterUpToSpeed(); })
      .BeforeStarting([this, speed] { currentVelocitySetpoint = speed(); },
                      {this});
}

frc2::CommandPtr ShooterSubsystem::GoToSpeedBasedOnGoal(
    std::function<units::meter_t()> distanceToGoal) {
  return GoToVelocityCmd(
      [this, distanceToGoal] { return lookupTable[distanceToGoal()]; },
      [] { return false; });
}

frc2::CommandPtr ShooterSubsystem::SysIdQuasistatic(
    frc2::sysid::Direction direction) {
  return sysIdRoutine.Quasistatic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

frc2::CommandPtr ShooterSubsystem::SysIdDynamic(
    frc2::sysid::Direction direction) {
  return sysIdRoutine.Dynamic(direction).BeforeStarting(
      [this] { ctre::phoenix6::SignalLogger::Start(); });
}

// This method will be called once per scheduler run
void ShooterSubsystem::Periodic() {
  ctre::phoenix6::BaseStatusSignal::RefreshAll(
      leftShooterVelSignal, rightShooterVelSignal, leftShooterPosSignal,
      leftShooterVoltageSignal);

  currentLeftPosition =
      leftShooterPosSignal.GetValue() * constants::shooter::SHOOTER_RATIO;
  currentLeftVoltage = leftShooterVoltageSignal.GetValue();
  currentLeftVelocity =
      ConvertMotorVelToShooterVel(leftShooterVelSignal.GetValue());
  currentRightVelocity =
      ConvertMotorVelToShooterVel(rightShooterVelSignal.GetValue());
}

void ShooterSubsystem::SimulationPeriodic() {
  leftShooterSim.SetInput(
      frc::Vectord<1>{leftShooterSimState.GetMotorVoltage().value()});
  rightShooterSim.SetInput(
      frc::Vectord<1>{rightShooterSimState.GetMotorVoltage().value()});

  leftShooterSim.Update(20_ms);
  rightShooterSim.Update(20_ms);

  leftShooterSimState.SetRawRotorPosition(leftShooterSim.GetAngularPosition());
  leftShooterSimState.SetRotorVelocity(
      ConvertShooterVelToMotorVel(leftShooterSim.GetAngularVelocity()));
  rightShooterSimState.SetRotorVelocity(
      ConvertShooterVelToMotorVel(rightShooterSim.GetAngularVelocity()));
}

void ShooterSubsystem::GoToVelocity(units::radians_per_second_t speed,
                                    bool dunker) {
  int slot = 0;
  if (dunker) {
    slot = 1;
  }

  currentVelocitySetpoint = speed;
  units::radians_per_second_t motorSetpoint =
      ConvertShooterVelToMotorVel(speed);
  shooterLeftMotor.SetControl(
      velocityControl.WithVelocity(motorSetpoint).WithSlot(slot));
  shooterRightMotor.SetControl(
      velocityControl.WithVelocity(motorSetpoint).WithSlot(slot));
}

void ShooterSubsystem::Set(double speed) {
  // This is because if we tell the motor to go to 0 rpm with velocity control,
  // it will spin in reverse, which we dont want
  currentVelocitySetpoint = speed * frc::DCMotor::Falcon500(1).freeSpeed;
  shooterLeftMotor.SetControl(dutyController.WithOutput(speed));
  shooterRightMotor.SetControl(dutyController.WithOutput(speed));
}

units::radians_per_second_t ShooterSubsystem::GetLeftShooterCurrentVelocity() {
  return currentLeftVelocity;
}

units::radians_per_second_t ShooterSubsystem::GetRightShooterCurrentVelocity() {
  return currentRightVelocity;
}

bool ShooterSubsystem::IsShooterUpToSpeed() {
  bool upToSpeed = (units::math::abs(currentVelocitySetpoint -
                                     GetLeftShooterCurrentVelocity()) <
                    constants::shooter::SHOOTER_TOLERANCE) &&
                   (units::math::abs(currentVelocitySetpoint -
                                     GetRightShooterCurrentVelocity()) <
                    constants::shooter::SHOOTER_TOLERANCE);
  return upToSpeed;
}

units::radians_per_second_t ShooterSubsystem::ConvertMotorVelToShooterVel(
    units::radians_per_second_t vel) {
  return vel * constants::shooter::SHOOTER_RATIO;
}

units::radians_per_second_t ShooterSubsystem::ConvertShooterVelToMotorVel(
    units::radians_per_second_t vel) {
  return vel / constants::shooter::SHOOTER_RATIO;
}

void ShooterSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;

  mainConfig.CurrentLimits.SupplyCurrentLimit = 40;
  mainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  mainConfig.Slot0.kP = currentGains.kP.to<double>();
  mainConfig.Slot0.kI = currentGains.kI.to<double>();
  mainConfig.Slot0.kD = currentGains.kD.to<double>();
  mainConfig.Slot0.kV = currentGains.kV.to<double>();
  mainConfig.Slot0.kA = currentGains.kA.to<double>();
  mainConfig.Slot0.kS = currentGains.kS.to<double>();

  mainConfig.Slot1.kP = constants::shooter::GAINS_DUNK.kP.to<double>();
  mainConfig.Slot1.kI = constants::shooter::GAINS_DUNK.kI.to<double>();
  mainConfig.Slot1.kD = constants::shooter::GAINS_DUNK.kD.to<double>();
  mainConfig.Slot1.kV = constants::shooter::GAINS_DUNK.kV.to<double>();
  mainConfig.Slot1.kA = constants::shooter::GAINS_DUNK.kA.to<double>();
  mainConfig.Slot1.kS = constants::shooter::GAINS_DUNK.kS.to<double>();

  mainConfig.MotorOutput.PeakReverseDutyCycle = 0;

  mainConfig.MotorOutput.Inverted = false;

  shooterLeftMotor.GetConfigurator().Apply(mainConfig);

  mainConfig.MotorOutput.Inverted = true;
  shooterRightMotor.GetConfigurator().Apply(mainConfig);

  leftShooterVoltageSignal.SetUpdateFrequency(100_Hz);
  leftShooterPosSignal.SetUpdateFrequency(100_Hz);
  leftShooterVelSignal.SetUpdateFrequency(100_Hz);
  rightShooterVelSignal.SetUpdateFrequency(100_Hz);
  shooterLeftMotor.OptimizeBusUtilization();
  shooterRightMotor.OptimizeBusUtilization();
}

void ShooterSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Velocity Setpoint (RPM)",
      [this] {
        return currentVelocitySetpoint.convert<units::revolutions_per_minute>()
            .value();
      },
      [this](double newSetpointRpm) {
        GoToVelocity(units::revolutions_per_minute_t{newSetpointRpm}, false);
      });
  builder.AddDoubleProperty(
      "Left Current Velocity (RPM)",
      [this] {
        return GetLeftShooterCurrentVelocity()
            .convert<units::revolutions_per_minute>()
            .value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "Right Current Velocity (RPM)",
      [this] {
        return GetRightShooterCurrentVelocity()
            .convert<units::revolutions_per_minute>()
            .value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kP = units::radian_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kI = units::radian_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kD = units::radian_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kV =
            units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit>{
                newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kA =
            units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit>{
                newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::shooter::ShooterGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
}

void ShooterSubsystem::SetGains(
    const constants::shooter::ShooterGains newGains) {
  currentGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kP = currentGains.kP.to<double>();
  newConfig.kI = currentGains.kI.to<double>();
  newConfig.kD = currentGains.kD.to<double>();
  newConfig.kV = currentGains.kV.to<double>();
  newConfig.kA = currentGains.kA.to<double>();
  newConfig.kS = currentGains.kS.to<double>();
  shooterLeftMotor.GetConfigurator().Apply(newConfig);
  shooterRightMotor.GetConfigurator().Apply(newConfig);
}

constants::shooter::ShooterGains ShooterSubsystem::GetGains() {
  return currentGains;
}
