// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/ClimberSubsystem.h"

#include <frc2/command/Commands.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "wpi/sendable/SendableBuilder.h"

ClimberSubsystem::ClimberSubsystem() {
  ConfigureMotor();

  frc::SmartDashboard::PutData("ClimberTelemetry", this);
  frc::SmartDashboard::PutData("Climber/ElevatorMech2d", &m_mech2d);
}

void ClimberSubsystem::ConfigureMotor() {
  leftClimberMotor.RestoreFactoryDefaults();
  rightClimberMotor.RestoreFactoryDefaults();

  leftClimberMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  rightClimberMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

  leftEncoder.SetPositionConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);
  leftEncoder.SetVelocityConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);

  rightEncoder.SetPositionConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);
  rightEncoder.SetVelocityConversionFactor(
      constants::climber::CLIMBER_SPOOL_RADIUS.value() *
      constants::climber::CLIMBER_RATIO);

  leftClimberMotor.BurnFlash();
  rightClimberMotor.BurnFlash();

  leftPIDController.SetP(currentGains.kP.value());
  leftPIDController.SetI(currentGains.kI.value());
  leftPIDController.SetD(currentGains.kD.value());

  rightPIDController.SetP(currentGains.kP.value());
  rightPIDController.SetI(currentGains.kI.value());
  rightPIDController.SetD(currentGains.kD.value());
}

frc2::CommandPtr ClimberSubsystem::ManualControl(
    std::function<double()> left, std::function<double()> right) {
  return frc2::cmd::RunEnd(
      [this, left, right] {
        leftClimberMotor.Set(left());
        rightClimberMotor.Set(right());
      },
      [this] {
        leftClimberMotor.Set(0);
        rightClimberMotor.Set(0);
      },
      {this});
}

// This method will be called once per scheduler run
void ClimberSubsystem::Periodic() {
  auto nextLeft = leftProfile.Calculate(20_ms, m_goal, leftSetpoint);
  auto nextRight = rightProfile.Calculate(20_ms, m_goal, rightSetpoint);
  ffResultLeft =
      ffLeft->Calculate(leftSetpoint.velocity, nextLeft.velocity, 20_ms);
  ffResultRight =
      ffRight->Calculate(rightSetpoint.velocity, nextRight.velocity, 20_ms);

  leftSetpoint = nextLeft;
  rightSetpoint = nextRight;

  m_elevatorLeftMech2d->SetLength(leftEncoder.GetPosition());
  m_elevatorRightMech2d->SetLength(rightEncoder.GetPosition());
}

void ClimberSubsystem::SimulationPeriodic() {
  leftElevatorSim.SetInput(
      frc::Vectord<1>{leftClimberMotor.GetAppliedOutput() *
                      frc::RobotController::GetInputVoltage()});

  leftElevatorSim.Update(20_ms);

  rightElevatorSim.SetInput(
      frc::Vectord<1>{rightClimberMotor.GetAppliedOutput() *
                      frc::RobotController::GetInputVoltage()});

  leftElevatorSim.Update(20_ms);
  rightElevatorSim.Update(20_ms);

  // update sim encoders here
  leftPos.Set(leftElevatorSim.GetPosition().value());
  rightPos.Set(rightElevatorSim.GetPosition().value());
}

units::meter_t ClimberSubsystem::GetLeftClimberHeight() {
  return units::meter_t{leftEncoder.GetPosition()};
}

units::meter_t ClimberSubsystem::GetRightClimberHeight() {
  return units::meter_t{rightEncoder.GetPosition()};
}

bool ClimberSubsystem::AreBothAtHeight() {
  if ((units::math::abs(GetLeftClimberHeight() - m_goal.position) <
       constants::climber::CLIMBER_TOLERANCE) &&
      (units::math::abs(GetRightClimberHeight() - m_goal.position) <
       constants::climber::CLIMBER_TOLERANCE)) {
    return true;
  } else {
    return false;
  }
}

void ClimberSubsystem::SetClimbHeight(units::meter_t newSetpoint) {
  m_goal.position = newSetpoint;
  m_goal.velocity = 0_mps;

  leftPIDController.SetReference(
      leftSetpoint.position.value(), rev::ControlType::kPosition, 0,
      ffResultLeft.value(), rev::CANPIDController::ArbFFUnits::kVoltage);
  rightPIDController.SetReference(
      rightSetpoint.position.value(), rev::ControlType::kPosition, 0,
      ffResultRight.value(), rev::CANPIDController::ArbFFUnits::kVoltage);
}

void ClimberSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Height Goal (Inches)",
      [this] { return m_goal.position.convert<units::inches>().value(); },
      [this](double newGoalInches) {
        SetClimbHeight(units::inch_t{newGoalInches});
      });
  builder.AddDoubleProperty(
      "Left Height Position (Inches)",
      [this] {
        return GetLeftClimberHeight().convert<units::inches>().value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "Right Height Position (Inches)",
      [this] {
        return GetRightClimberHeight().convert<units::inches>().value();
      },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kP = units::meter_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kI = units::meter_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kD = units::meter_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kV = units::unit_t<frc::ElevatorFeedforward::kv_unit>{newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kA = units::unit_t<frc::ElevatorFeedforward::ka_unit>{newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kG", [this] { return currentGains.kG.to<double>(); },
      [this](double newKg) {
        constants::climber::ClimberGains newGains = GetGains();
        newGains.kG = units::volt_t{newKg};
        SetGains(newGains);
      });
}

void ClimberSubsystem::SetGains(
    const constants::climber::ClimberGains newGains) {
  currentGains = newGains;
  ffLeft = std::make_unique<frc::ElevatorFeedforward>(newGains.kS, newGains.kG,
                                                      newGains.kV, newGains.kA);
  ffRight = std::make_unique<frc::ElevatorFeedforward>(
      newGains.kS, newGains.kG, newGains.kV, newGains.kA);
  leftPIDController.SetP(newGains.kP.value());
  leftPIDController.SetI(newGains.kI.value());
  leftPIDController.SetD(newGains.kD.value());
  rightPIDController.SetP(newGains.kP.value());
  rightPIDController.SetI(newGains.kI.value());
  rightPIDController.SetD(newGains.kD.value());
}

constants::climber::ClimberGains ClimberSubsystem::GetGains() {
  return currentGains;
}
