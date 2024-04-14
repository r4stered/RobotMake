// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/IntakeSubsystem.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

IntakeSubsystem::IntakeSubsystem() {
  ConfigureMotors();

  intakeSensor.SetRangingMode(frc::TimeOfFlight::RangingMode::kShort, 24);
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
  units::millimeter_t intakeDist = units::millimeter_t{intakeSensor.GetRange()};
  filteredReading = intakeFilter.Calculate(intakeDist);
  frc::SmartDashboard::PutNumber("Intake/Sensor Distance",
                                 intakeDist.convert<units::inches>().value());
  frc::SmartDashboard::PutNumber("Intake/Filtered Sensor Distance",
                                 filteredReading.value());
  frc::SmartDashboard::PutNumber("Intake/Motor Speed", intakeMotor.Get());
}

void IntakeSubsystem::ConfigureMotors() {
  ctre::phoenix6::configs::TalonFXConfiguration mainConfig;

  mainConfig.CurrentLimits.SupplyCurrentLimit = 40;
  mainConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  mainConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  mainConfig.MotorOutput.Inverted = true;

  intakeMotor.GetConfigurator().Apply(mainConfig);

  ctre::phoenix6::configs::TalonFXConfiguration followConfig;

  followConfig.CurrentLimits.SupplyCurrentLimit = 40;
  followConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

  followConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;

  intakeSecondMotor.GetConfigurator().Apply(followConfig);
}

frc2::CommandPtr IntakeSubsystem::SuckInNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(1); },
                           [this] { SetIntakeSpeed(0); }, {this})
      .BeforeStarting(
          [this] {
#ifndef __FRC_ROBORIO__
            if (SeesNote()) {
              intakeSensor.SetDistance(12_in);
            } else {
              intakeSensor.SetDistance(0_in);
            }
#endif
          },
          {this});
}

frc2::CommandPtr IntakeSubsystem::SpitOutNotes() {
  return frc2::cmd::RunEnd([this] { SetIntakeSpeed(-1); },
                           [this] { SetIntakeSpeed(0); }, {this});
}

frc2::CommandPtr IntakeSubsystem::SuckInUntilNoteIsSeen() {
  return SuckInNotes().Until([this] { return SeesNote(); });
}

bool IntakeSubsystem::SeesNote() {
  return units::millimeter_t{intakeSensor.GetRange()} <=
         constants::intake::INTAKE_SENSOR_DISTANCE;
}

bool IntakeSubsystem::SensorFoundEdge() {
  return filteredReading <= constants::intake::INTAKE_SENSOR_DISTANCE;
}

void IntakeSubsystem::SetIntakeSpeed(double speed) {
  intakeMotor.Set(speed);
  intakeSecondMotor.SetControl(
      ctre::phoenix6::controls::Follower(intakeMotor.GetDeviceID(), true));
}
