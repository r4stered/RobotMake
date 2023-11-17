// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveModule.h"

#include <frc/DataLogManager.h>

#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"
#include "str/Units.h"

SwerveModule::SwerveModule(int driveMotorId, int steerMotorId, int steerEncId,
  double steerEncOffset, bool invertDrive, bool invertSteer)
  : driveMotor{driveMotorId, "*"}
  , steerMotor{steerMotorId, "*"}
  , steerEnc{steerEncId, "*"}
  , currentSteeringGains{constants::drivebase::gains::STEER_KA,
      constants::drivebase::gains::STEER_KV,
      constants::drivebase::gains::STEER_KS,
      constants::drivebase::gains::STEER_KP,
      constants::drivebase::gains::STEER_KI,
      constants::drivebase::gains::STEER_KD}
  , currentDrivingGains{
      constants::drivebase::gains::DRIVE_KA,
      constants::drivebase::gains::DRIVE_KV,
      constants::drivebase::gains::DRIVE_KS,
      constants::drivebase::gains::DRIVE_KP,
      constants::drivebase::gains::DRIVE_KI,
      constants::drivebase::gains::DRIVE_KD,
    }
{
  ConfigureDriveMotor(invertDrive);
  ConfigureSteerEncoder(steerEncOffset);
  ConfigureSteerMotor(invertSteer);

  velocityTorqueSetter.UpdateFreqHz = 0_Hz;
  voltageOpenLoopSetter.UpdateFreqHz = 0_Hz;
  angleSetter.UpdateFreqHz = 0_Hz;
}

void SwerveModule::ConfigureDriveMotor(bool invertDrive)
{
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};

  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = currentDrivingGains.kV;
  driveSlotConfig.kA = currentDrivingGains.kA;
  driveSlotConfig.kS = currentDrivingGains.kS;
  driveSlotConfig.kP = currentDrivingGains.kP;
  driveSlotConfig.kI = currentDrivingGains.kI;
  driveSlotConfig.kD = currentDrivingGains.kD;
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode
    = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Feedback.SensorToMechanismRatio
    = constants::drivebase::physical::DRIVE_GEARING;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent
    = constants::drivebase::physical::SLIP_CURRENT.to<double>();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent
    = -constants::drivebase::physical::SLIP_CURRENT.to<double>();
  driveConfig.CurrentLimits.StatorCurrentLimit
    = constants::drivebase::physical::SLIP_CURRENT.to<double>();
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted = invertDrive
    ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
    : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode status
    = driveMotor.GetConfigurator().Apply(driveConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
      "Swerve Module ConfigureDriveMotor() status wasn't ok it was: "
      "{}. More info: {}",
      status.GetName(), status.GetDescription()));
  } else {
    frc::DataLogManager::Log(
      "Swerve Module ConfigureDriveMotor() status was OK");
  }
}

void SwerveModule::ConfigureSteerEncoder(double encOffset)
{
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encOffset;
  ctre::phoenix::StatusCode status
    = steerEnc.GetConfigurator().Apply(encoderConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
      "Swerve Module ConfigureSteerEncoder() status wasn't ok it was: "
      "{}. More info: {}",
      status.GetName(), status.GetDescription()));
  } else {
    frc::DataLogManager::Log(
      "Swerve Module ConfigureSteerEncoder() status was OK");
  }
}

void SwerveModule::ConfigureSteerMotor(bool invertSteer)
{
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};

  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = currentSteeringGains.kV;
  steerSlotConfig.kA = currentSteeringGains.kA;
  steerSlotConfig.kS = currentSteeringGains.kS;
  steerSlotConfig.kP = currentSteeringGains.kP;
  steerSlotConfig.kI = currentSteeringGains.kI;
  steerSlotConfig.kD = currentSteeringGains.kD;
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode
    = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEnc.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource
    = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio
    = constants::drivebase::physical::STEER_GEARING;
  steerConfig.MotorOutput.Inverted = invertSteer
    ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
    : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.MotionMagic.MotionMagicAcceleration
    = constants::drivebase::gains::STEER_MOTION_MAGIC_ACCEL;
  steerConfig.MotionMagic.MotionMagicCruiseVelocity
    = constants::drivebase::gains::STEER_MOTION_MAGIC_CRUISE_VEL;
  ctre::phoenix::StatusCode status
    = steerMotor.GetConfigurator().Apply(steerConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
      "Swerve Module ConfigureSteerMotor() status wasn't ok it was: "
      "{}. More info: {}",
      status.GetName(), status.GetDescription()));
  } else {
    frc::DataLogManager::Log(
      "Swerve Module ConfigureSteerMotor() status was OK");
  }
}

void SwerveModule::SetSteeringGains(const ModuleGains& newGains)
{
  currentSteeringGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kA = currentSteeringGains.kA;
  newConfig.kV = currentSteeringGains.kV;
  newConfig.kS = currentSteeringGains.kS;
  newConfig.kP = currentSteeringGains.kP;
  newConfig.kI = currentSteeringGains.kI;
  newConfig.kD = currentSteeringGains.kD;
  steerMotor.GetConfigurator().Apply(newConfig);
}

ModuleGains SwerveModule::GetSteeringGains() { return currentSteeringGains; }

void SwerveModule::SetDrivingGains(const ModuleGains& newGains)
{
  currentDrivingGains = newGains;
  ctre::phoenix6::configs::Slot0Configs newConfig{};
  newConfig.kA = currentDrivingGains.kA;
  newConfig.kV = currentDrivingGains.kV;
  newConfig.kS = currentDrivingGains.kS;
  newConfig.kP = currentDrivingGains.kP;
  newConfig.kI = currentDrivingGains.kI;
  newConfig.kD = currentDrivingGains.kD;
  driveMotor.GetConfigurator().Apply(newConfig);
}

ModuleGains SwerveModule::GetDrivingGains() { return currentDrivingGains; }

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh)
{
  if (refresh) {
    ctre::phoenix::StatusCode status
      = ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, steerAngleSig,
        steerAngleVelSig, drivePositionSig, driveVelocitySig);
    if (!status.IsOK()) {
      frc::DataLogManager::Log(fmt::format(
        "Swerve Module WaitForAll() status wasn't ok it was: {}. More info: {}",
        status.GetName(), status.GetDescription()));
    }
  }

  units::turn_t driveTurns
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      drivePositionSig, driveVelocitySig);
  units::turn_t steerTurns
    = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      steerAngleSig, steerAngleVelSig);

  // This compensates for when the drive wheel rotates a little when you rotate
  // the module
  driveTurns = driveTurns
    - (steerTurns * constants::drivebase::physical::DRIVE_STEER_COUPLING);

  internalState = frc::SwerveModulePosition{
    str::Units::ConvertAngularDistanceToLinearDistance(
      driveTurns, constants::drivebase::physical::WHEEL_DIAM / 2),
    frc::Rotation2d{steerTurns.convert<units::radians>()}};
  return internalState;
}

void SwerveModule::GoToState(frc::SwerveModuleState state, bool openLoop)
{
  frc::SwerveModuleState optimizedState
    = frc::SwerveModuleState::Optimize(state, internalState.angle);

  // Reverses the expected module shimmy when rotating
  units::turns_per_second_t moduleTurnSpeed = steerAngleVelSig.GetValue();
  units::turns_per_second_t driveRateBackout
    = moduleTurnSpeed * constants::drivebase::physical::DRIVE_STEER_COUPLING;
  units::turns_per_second_t velocityToGoTo
    = str::Units::ConvertLinearVelocityToAngularVelocity(
        optimizedState.speed, constants::drivebase::physical::WHEEL_DIAM / 2)
    - driveRateBackout;

  steerMotor.SetControl(
    angleSetter.WithPosition(optimizedState.angle.Radians()));
  if (openLoop) {
    driveMotor.SetControl(voltageOpenLoopSetter.WithOutput(
      (velocityToGoTo
        / str::Units::ConvertLinearVelocityToAngularVelocity(
          constants::drivebase::physical::MAX_DRIVE_SPEED,
          constants::drivebase::physical::WHEEL_DIAM / 2))
      * 12.0_V));
  } else {
    driveMotor.SetControl(velocityTorqueSetter.WithVelocity(velocityToGoTo));
  }
}

frc::SwerveModulePosition SwerveModule::GetCachedPosition()
{
  return internalState;
}

frc::SwerveModuleState SwerveModule::GetCurrentState()
{
  return frc::SwerveModuleState{
    str::Units::ConvertAngularVelocityToLinearVelocity(
      driveVelocitySig.GetValue(),
      constants::drivebase::physical::WHEEL_DIAM / 2),
    frc::Rotation2d{steerAngleSig.GetValue().convert<units::radians>()}};
}

void SwerveModule::ResetPosition() { driveMotor.SetPosition(0_tr); }

std::array<ctre::phoenix6::BaseStatusSignal*, 4> SwerveModule::GetSignals()
{
  return {
    &drivePositionSig, &driveVelocitySig, &steerAngleSig, &steerAngleVelSig};
}

void SwerveModule::SetSteerMotorVolts(units::volt_t voltage)
{
  steerMotor.SetControl(identifySteerSetter.WithOutput(voltage));
}

CharData SwerveModule::GetCharData()
{
  ctre::phoenix::StatusCode status
    = ctre::phoenix6::BaseStatusSignal::WaitForAll(
      0_s, steerAngleSig, steerAngleVelSig, steerVoltageSig);

  units::volt_t motorVoltage = steerVoltageSig.GetValue();
  units::radian_t steerAngle = steerAngleSig.GetValue();
  units::radians_per_second_t steerAngleVel = steerAngleVelSig.GetValue();

  return CharData{motorVoltage, steerAngle, steerAngleVel};
}

void SwerveModule::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("SwerveModule");
  builder.AddDoubleProperty(
    "Drive kA", [this] { return currentDrivingGains.kA; },
    [this](double newKa) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kA = newKa;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kA", [this] { return currentDrivingGains.kA; },
    [this](double newKa) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kA = newKa;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kV", [this] { return currentDrivingGains.kV; },
    [this](double newKv) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kV = newKv;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kS", [this] { return currentDrivingGains.kS; },
    [this](double newKs) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kS = newKs;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kP", [this] { return currentDrivingGains.kP; },
    [this](double newKp) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kP = newKp;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kI", [this] { return currentDrivingGains.kI; },
    [this](double newKi) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kI = newKi;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Drive kD", [this] { return currentDrivingGains.kD; },
    [this](double newKd) {
      ModuleGains newGains = GetDrivingGains();
      newGains.kD = newKd;
      SetDrivingGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kA", [this] { return currentSteeringGains.kA; },
    [this](double newKa) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kA = newKa;
      SetSteeringGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kV", [this] { return currentSteeringGains.kV; },
    [this](double newKv) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kV = newKv;
      SetSteeringGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kS", [this] { return currentSteeringGains.kS; },
    [this](double newKs) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kS = newKs;
      SetSteeringGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kP", [this] { return currentSteeringGains.kP; },
    [this](double newKp) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kP = newKp;
      SetSteeringGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kI", [this] { return currentSteeringGains.kI; },
    [this](double newKi) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kI = newKi;
      SetSteeringGains(newGains);
    });
  builder.AddDoubleProperty(
    "Steer kD", [this] { return currentSteeringGains.kD; },
    [this](double newKd) {
      ModuleGains newGains = GetSteeringGains();
      newGains.kD = newKd;
      SetSteeringGains(newGains);
    });
}
