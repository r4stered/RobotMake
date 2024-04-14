// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/DunkerSubsystem.h"

#include "frc/smartdashboard/SmartDashboard.h"

DunkerSubsystem::DunkerSubsystem() {
  ConfigureMotors();
  frc::SmartDashboard::PutData("Dunker Telemetry", this);
  frc::SmartDashboard::PutData("Dunker Mechanism Vis", &pivotMech);
  pivotController.SetTolerance(constants::dunker::DUNKER_PIVOT_ANGLE_TOLERANCE,
                               constants::dunker::DUNKER_PIVOT_VEL_TOLERANCE);

  SetPivotGoal(22_deg);
}

frc2::CommandPtr DunkerSubsystem::PivotDunkNotesOut() {
  return frc2::cmd::Sequence(frc2::cmd::RunOnce(
      [this] { SetPivotGoal(constants::dunker::DUNKER_OUT_ANGLE); }, {this}));
}

frc2::CommandPtr DunkerSubsystem::PivotDunkNotesIn() {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] { SetPivotGoal(constants::dunker::DUNKER_IN_ANGLE); }, {this}),
      frc2::cmd::WaitUntil([this] { return IsPivotAtGoal(); }));
}

frc2::CommandPtr DunkerSubsystem::DunkManual(std::function<double()> speed) {
  return frc2::cmd::RunOnce([this, speed] { SetDunkSpeed(speed()); }, {this});
}

frc2::CommandPtr DunkerSubsystem::DunkTheNotes() {
  return frc2::cmd::RunEnd([this] { SetDunkSpeed(1); },
                           [this] { SetDunkSpeed(0); }, {this});
}
frc2::CommandPtr DunkerSubsystem::StopDunking() {
  return frc2::cmd::RunOnce([this] { SetDunkSpeed(0); }, {this});
}

frc2::CommandPtr DunkerSubsystem::JammedDunkNotes() {
  return frc2::cmd::RunEnd([this] { SetDunkSpeed(-1); },
                           [this] { SetDunkSpeed(0); }, {this});
}

frc2::CommandPtr DunkerSubsystem::SysIdQuasistatic(
    frc2::sysid::Direction direction) {
  return sysIdRoutine.Quasistatic(direction).BeforeStarting(
      [this] { currentlyCharacterizing = true; });
}

frc2::CommandPtr DunkerSubsystem::SysIdDynamic(
    frc2::sysid::Direction direction) {
  return sysIdRoutine.Dynamic(direction).BeforeStarting(
      [this] { currentlyCharacterizing = true; });
}

void DunkerSubsystem::Periodic() {
  double encReading = pivotEncoder.GetOutput();
  if (encReading < .5) {
    encReading = .99;
  }
  frc::SmartDashboard::PutNumber("Dunker/EncoderPos", encReading);
  currentPivotPos = ConvertEncoderToAngle(encReading);

  pivotVelocity = (currentPivotPos - prevPivotPosition) / 0.02_s;
  prevPivotPosition = currentPivotPos;

  auto ff = pivotFeedforward->Calculate(pivotController.GetSetpoint().position,
                                        pivotController.GetSetpoint().velocity);

  if (!currentlyCharacterizing) {
    dunkPivotMotor.SetVoltage(
        units::volt_t{pivotController.Calculate(currentPivotPos)} + ff);
  }
}

void DunkerSubsystem::SimulationPeriodic() {
  pivotSim.SetInput(frc::Vectord<1>{dunkPivotMotor.Get() *
                                    frc::RobotController::GetInputVoltage()});
  pivotSim.Update(20_ms);
  encoderSim.SetOutput(ConvertAngleToEncoder(pivotSim.GetAngle()));
  pivotArm->SetAngle(-pivotSim.GetAngle() + 180_deg);
}

void DunkerSubsystem::ConfigureMotors() {
  dunkPivotMotor.RestoreFactoryDefaults();
  dunkPivotMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
  dunkPivotMotor.SetSmartCurrentLimit(40);
  dunkPivotMotor.SetInverted(false);
  if (dunkPivotMotor.BurnFlash() == rev::REVLibError::kOk) {
    fmt::print("Successfully configured dunk pivot motor!\n");
  } else {
    fmt::print("ERROR: Unable to configure dunk pivot motor!\n");
  }

  ctre::phoenix6::configs::TalonFXConfiguration dunkMotorConfig;
  dunkMotorConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Coast;
  dunkMotorConfig.CurrentLimits.SupplyCurrentLimit = 30;
  dunkMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  dunkMotorConfig.MotorOutput.Inverted = true;
  dunkMotor.GetConfigurator().Apply(dunkMotorConfig);
}

units::radian_t DunkerSubsystem::ConvertEncoderToAngle(double encoderReading) {
  return units::radian_t{
      units::Map<double>(encoderReading, constants::dunker::DUNKER_MIN_ENCODER,
                         constants::dunker::DUNKER_MAX_ENCODER,
                         constants::dunker::DUNKER_MIN_ANGLE.value(),
                         constants::dunker::DUNKER_MAX_ANGLE.value())};
}

double DunkerSubsystem::ConvertAngleToEncoder(units::radian_t angle) {
  return units::Map<double>(angle.value(),
                            constants::dunker::DUNKER_MIN_ANGLE.value(),
                            constants::dunker::DUNKER_MAX_ANGLE.value(),
                            constants::dunker::DUNKER_MIN_ENCODER,
                            constants::dunker::DUNKER_MAX_ENCODER);
}

units::radian_t DunkerSubsystem::GetPivotAngle() {
  return currentPivotPos;
}

void DunkerSubsystem::SetDunkSpeed(double speed) {
  fmt::print("speed: {}\n", speed);
  dunkMotor.SetControl(dutyController.WithOutput(speed));
}

void DunkerSubsystem::SetPivotGoal(units::radian_t angleGoal) {
  pivotController.SetGoal(angleGoal);
}

bool DunkerSubsystem::IsPivotAtGoal() {
  return pivotController.AtGoal();
}

void DunkerSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  frc2::SubsystemBase::InitSendable(builder);
  builder.AddDoubleProperty(
      "Angle Goal (Degrees)",
      [this] {
        return pivotController.GetGoal()
            .position.convert<units::degrees>()
            .value();
      },
      [this](double newGoalDegrees) {
        SetPivotGoal(units::degree_t{newGoalDegrees});
      });
  builder.AddDoubleProperty(
      "Current Position (Degrees)",
      [this] { return GetPivotAngle().convert<units::degrees>().value(); },
      nullptr);
  builder.AddDoubleProperty(
      "kP", [this] { return currentGains.kP.to<double>(); },
      [this](double newKp) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kP = units::radian_volt_kp_unit_t{newKp};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kI", [this] { return currentGains.kI.to<double>(); },
      [this](double newKi) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kI = units::radian_volt_ki_unit_t{newKi};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kD", [this] { return currentGains.kD.to<double>(); },
      [this](double newKd) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kD = units::radian_volt_kd_unit_t{newKd};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kV", [this] { return currentGains.kV.to<double>(); },
      [this](double newKv) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kV =
            units::unit_t<frc::SimpleMotorFeedforward<units::radians>::kv_unit>{
                newKv};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kA", [this] { return currentGains.kA.to<double>(); },
      [this](double newKa) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kA =
            units::unit_t<frc::SimpleMotorFeedforward<units::radians>::ka_unit>{
                newKa};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kS", [this] { return currentGains.kS.to<double>(); },
      [this](double newKs) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kS = units::volt_t{newKs};
        SetGains(newGains);
      });
  builder.AddDoubleProperty(
      "kG", [this] { return currentGains.kG.to<double>(); },
      [this](double newKg) {
        constants::dunker::DunkerGains newGains = GetGains();
        newGains.kG = units::volt_t{newKg};
        SetGains(newGains);
      });
}

void DunkerSubsystem::SetGains(const constants::dunker::DunkerGains newGains) {
  currentGains = newGains;
  pivotFeedforward = std::make_unique<frc::ArmFeedforward>(
      newGains.kS, newGains.kG, newGains.kV, newGains.kA);
  pivotController.SetPID(newGains.kP.value(), newGains.kI.value(),
                         newGains.kD.value());
}

constants::dunker::DunkerGains DunkerSubsystem::GetGains() {
  return currentGains;
}
