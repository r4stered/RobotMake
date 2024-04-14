// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveModule.h"

#include <ctre/phoenix/StatusCodes.h>
#include <frc/DataLogManager.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include <iostream>

#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "Constants.h"
#include "frc/MathUtil.h"
#include "frc/RobotBase.h"
#include "frc/RobotController.h"
#include "frc/smartdashboard/SmartDashboard.h"

using namespace str;

SwerveModule::SwerveModule(const SwerveModuleConstants& moduleConstants)
    : driveMotor(moduleConstants.driveMotorCanId, "*"),
      steerMotor(moduleConstants.steerMotorCanId, "*"),
      steerEncoder(moduleConstants.steerEncoderCanId, "*") {
  ctre::phoenix::StatusCode driveConfigCode = ConfigureDriveMotor(
      frc::RobotBase::IsReal() ? moduleConstants.invertDrive : false);
  if (!driveConfigCode.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Module ConfigureDriveMotor wasn't ok it was: {}. "
        "More info: {}",
        driveConfigCode.GetName(), driveConfigCode.GetDescription()));
  }
  ctre::phoenix::StatusCode steerConfigCode = ConfigureSteerMotor(
      frc::RobotBase::IsReal() ? moduleConstants.invertSteer : false);
  if (!steerConfigCode.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Module ConfigureSteerMotor wasn't ok it was: {}. "
        "More info: {}",
        steerConfigCode.GetName(), steerConfigCode.GetDescription()));
  }
  ctre::phoenix::StatusCode steerEncoderCode = ConfigureSteerEncoder(
      frc::RobotBase::IsReal() ? moduleConstants.steerEncoderOffset : 0.0);
  if (!steerEncoderCode.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Module ConfigureSteerEncoder wasn't ok it was: {}. "
        "More info: {}",
        steerEncoderCode.GetName(), steerEncoderCode.GetDescription()));
  }

  driveVelocitySetter = driveVelocitySetter.WithOverrideCoastDurNeutral(true);

  driveVelocitySetter.UpdateFreqHz = 0_Hz;
  driveVoltageSetter.UpdateFreqHz = 0_Hz;
  driveTorqueSetter.UpdateFreqHz = 0_Hz;
  steerAngleSetter.UpdateFreqHz = 0_Hz;
  steerTorqueSetter.UpdateFreqHz = 0_Hz;
}

void SwerveModule::SimulationUpdate(units::meter_t driveDistance,
                                    units::meters_per_second_t driveVelocity,
                                    units::radian_t steerDistance,
                                    units::radians_per_second_t steerVelocity) {
  units::radian_t converted =
      ConvertWheelDistanceToMotorShaftRotations(driveDistance);
  simDriveMotor.SetRawRotorPosition(converted);
  simDriveMotor.SetRotorVelocity(
      ConvertWheelVelocityToMotorVelocity(driveVelocity));
  simDriveMotor.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  simSteerMotor.SetRawRotorPosition(
      ConvertOutputShaftPositionToMotorShaftPosition(steerDistance));
  simSteerMotor.SetRotorVelocity(
      ConvertOutputShaftVelocityToMotorShaftVelocity(steerVelocity));
  simSteerMotor.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  simEncoder.SetRawPosition(steerDistance);
  simEncoder.SetVelocity(steerVelocity);
  simEncoder.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh) {
  if (refresh) {
    ctre::phoenix::StatusCode status =
        ctre::phoenix6::BaseStatusSignal::WaitForAll(
            0_s, steerAngleSignal, steerAngleVelocitySignal, steerCurrentSignal,
            drivePositionSignal, driveVelocitySignal, driveCurrentSignal);
    if (!status.IsOK()) {
      frc::DataLogManager::Log(
          fmt::format("Swerve Module GetPosition WaitForAll() status wasn't ok "
                      "it was: {}. More Info: {}",
                      status.GetName(), status.GetDescription()));
    }
  }

  units::radian_t drivePosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSignal, driveVelocitySignal);

  units::radian_t steerPosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          steerAngleSignal, steerAngleVelocitySignal);

  drivePosition =
      drivePosition -
      (steerPosition * constants::swerve::physical::DRIVE_STEER_COUPLING);

  currentPosition =
      frc::SwerveModulePosition{ConvertMotorToWheelDistance(drivePosition),
                                frc::Rotation2d{steerPosition}};

  currentState = frc::SwerveModuleState{
      ConvertMotorSpeedToWheelVelocity(driveVelocitySignal.GetValue()),
      frc::Rotation2d{steerPosition}};

  return currentPosition;
}

frc::SwerveModulePosition SwerveModule::GetCachedPosition() const {
  return currentPosition;
}

frc::SwerveModuleState SwerveModule::GetState() const {
  return currentState;
}

void SwerveModule::GoToState(const frc::SwerveModuleState& state, bool openLoop,
                             bool optimize) {
  frc::SwerveModuleState stateToGoTo = state;
  if (optimize) {
    stateToGoTo = frc::SwerveModuleState::Optimize(state, currentState.angle);
  }

  units::radians_per_second_t moduleTurnSpeed =
      steerAngleVelocitySignal.GetValue();
  units::radians_per_second_t driveRateBackout =
      moduleTurnSpeed * constants::swerve::physical::DRIVE_STEER_COUPLING;

  units::radians_per_second_t velocityToGoTo =
      ConvertWheelVelocityToMotorVelocity(stateToGoTo.speed);

  units::radian_t steerMotorError =
      stateToGoTo.angle.Radians() - steerAngleSignal.GetValue();
  units::scalar_t cosineScalar = units::math::cos(steerMotorError);
  if (cosineScalar < 0.0) {
    cosineScalar = 0.0;
  }
  velocityToGoTo = velocityToGoTo * cosineScalar;
  velocityToGoTo = velocityToGoTo + driveRateBackout;

  steerMotor.SetControl(
      steerAngleSetter.WithPosition(stateToGoTo.angle.Radians()));
  if (openLoop) {
    driveMotor.SetControl(driveVoltageSetter.WithOutput(
        (velocityToGoTo / ConvertWheelVelocityToMotorVelocity(
                              constants::swerve::physical::MAX_LINEAR_SPEED)) *
        12_V));
  } else {
    driveMotor.SetControl(driveVelocitySetter.WithVelocity(velocityToGoTo));
  }

  currentAngleSetpoint = stateToGoTo.angle.Radians();
  currentDriveSetpoint = stateToGoTo.speed;
}

void SwerveModule::LockSteerAtZero() {
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      currentState.angle);
  steerMotor.SetControl(steerAngleSetter.WithPosition(0_rad));
}

void SwerveModule::PushMode(bool onOff) {
  if (onOff) {
    driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
  } else {
    driveMotor.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  }
  LockSteerAtZero();
}

units::radian_t SwerveModule::GetMotorRotations() {
  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, drivePositionSignal,
                                                   driveVelocitySignal);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Module GetMotorRotations WaitForAll() status wasn't ok "
        "it was: {}. More Info: {}",
        status.GetName(), status.GetDescription()));
  }

  units::radian_t drivePosition =
      ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
          drivePositionSignal, driveVelocitySignal);
  return drivePosition;
}

void SwerveModule::SetModuleSteerGains(
    const constants::swerve::ModuleSteerGains& newGains) {
  currentSteeringGains = newGains;
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = currentSteeringGains.kV.value();
  steerSlotConfig.kA = currentSteeringGains.kA.value();
  steerSlotConfig.kS = currentSteeringGains.kS.value();
  steerSlotConfig.kP = currentSteeringGains.kP;
  steerSlotConfig.kI = currentSteeringGains.kI;
  steerSlotConfig.kD = currentSteeringGains.kD;
  ctre::phoenix::StatusCode status =
      steerMotor.GetConfigurator().Apply(steerSlotConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Swerve Steer Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    status.GetName(), status.GetDescription()));
  }
}

constants::swerve::ModuleSteerGains SwerveModule::GetCurrentSteerGains() const {
  return currentSteeringGains;
}

void SwerveModule::SetModuleDriveGains(
    const constants::swerve::ModuleDriveGains& newGains) {
  currentDrivingGains = newGains;
  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = currentDrivingGains.kV.value();
  driveSlotConfig.kA = currentDrivingGains.kA.value();
  driveSlotConfig.kS = currentDrivingGains.kS.value();
  driveSlotConfig.kP = currentDrivingGains.kP;
  driveSlotConfig.kI = currentDrivingGains.kI;
  driveSlotConfig.kD = currentDrivingGains.kD;
  ctre::phoenix::StatusCode status =
      driveMotor.GetConfigurator().Apply(driveSlotConfig);
  if (!status.IsOK()) {
    frc::DataLogManager::Log(
        fmt::format("Swerve Drive Motor was unable to set new gains! "
                    "Error: {}, More Info: {}",
                    status.GetName(), status.GetDescription()));
  }
}

constants::swerve::ModuleDriveGains SwerveModule::GetCurrentDriveGains() const {
  return currentDrivingGains;
}

void SwerveModule::ResetPosition() {
  driveMotor.SetPosition(0_rad);
}

std::array<ctre::phoenix6::BaseStatusSignal*, 8> SwerveModule::GetSignals() {
  return {&steerAngleSignal,   &steerAngleVelocitySignal, &steerVoltageSignal,
          &steerCurrentSignal, &drivePositionSignal,      &driveVelocitySignal,
          &driveVoltageSignal, &driveCurrentSignal};
}

void SwerveModule::OptimizeBusSignals() {
  auto driveStatus = driveMotor.OptimizeBusUtilization();
  if (!driveStatus.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Drive Motor was unable to optimize its bus signals! "
        "Error: {}, More Info: {}",
        driveStatus.GetName(), driveStatus.GetDescription()));
  }
  auto steerStatus = steerMotor.OptimizeBusUtilization();
  if (!steerStatus.IsOK()) {
    frc::DataLogManager::Log(fmt::format(
        "Swerve Steer Motor was unable to optimize its bus signals! "
        "Error: {}, More Info: {}",
        steerStatus.GetName(), steerStatus.GetDescription()));
  }
}

ctre::phoenix::StatusCode SwerveModule::ConfigureDriveMotor(bool invertDrive) {
  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};

  ctre::phoenix6::configs::Slot0Configs driveSlotConfig{};
  driveSlotConfig.kV = currentDrivingGains.kV.value();
  driveSlotConfig.kA = currentDrivingGains.kA.value();
  driveSlotConfig.kS = currentDrivingGains.kS.value();
  driveSlotConfig.kP = currentDrivingGains.kP;
  driveSlotConfig.kI = currentDrivingGains.kI;
  driveSlotConfig.kD = currentDrivingGains.kD;
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
      constants::swerve::physical::SLIP_CURRENT.value();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
      -constants::swerve::physical::SLIP_CURRENT.value();
  driveConfig.CurrentLimits.StatorCurrentLimit =
      constants::swerve::physical::SLIP_CURRENT.value();
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.MotorOutput.Inverted =
      invertDrive
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit =
      constants::swerve::physical::SUPPLY_CURRENT_LIMIT.value();

  return driveMotor.GetConfigurator().Apply(driveConfig);
}

ctre::phoenix::StatusCode SwerveModule::ConfigureSteerMotor(bool invertSteer) {
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};

  // Be careful here. Make sure we are controlling in volts as gains will be
  // different depending on control mode.
  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV =
      frc::RobotBase::IsReal() ? currentSteeringGains.kV.value() : 0.0;
  steerSlotConfig.kA =
      frc::RobotBase::IsReal() ? currentSteeringGains.kA.value() : 0.0;
  steerSlotConfig.kS =
      frc::RobotBase::IsReal() ? currentSteeringGains.kS.value() : 0.0;
  steerSlotConfig.kP =
      frc::RobotBase::IsReal() ? currentSteeringGains.kP : units::scalar_t{300};
  steerSlotConfig.kI =
      frc::RobotBase::IsReal() ? currentSteeringGains.kI : units::scalar_t{0};
  steerSlotConfig.kD =
      frc::RobotBase::IsReal() ? currentSteeringGains.kD : units::scalar_t{50};
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode =
      ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEncoder.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource =
      ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio =
      constants::swerve::physical::STEER_GEARING;
  steerConfig.MotorOutput.Inverted =
      invertSteer
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.MotionMagic.MotionMagicCruiseVelocity =
      100.0 / constants::swerve::physical::STEER_GEARING;
  steerConfig.MotionMagic.MotionMagicAcceleration =
      steerConfig.MotionMagic.MotionMagicCruiseVelocity /
      static_cast<double>(0.100);
  steerConfig.MotionMagic.MotionMagicExpo_kV =
      0.12 * constants::swerve::physical::STEER_GEARING;
  steerConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit =
      constants::swerve::physical::SUPPLY_CURRENT_LIMIT.value();

  return steerMotor.GetConfigurator().Apply(steerConfig);
}

ctre::phoenix::StatusCode SwerveModule::ConfigureSteerEncoder(
    double encoderOffset) {
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encoderOffset;
  return steerEncoder.GetConfigurator().Apply(encoderConfig);
}

void SwerveModule::Log(int moduleIndex) {
  frc::SwerveModuleState state = GetState();

  std::string table = "Drivebase/Module " + std::to_string(moduleIndex) + "/";
  frc::SmartDashboard::PutNumber(table + "Steer Degrees",
                                 frc::AngleModulus(state.angle.Radians())
                                     .convert<units::degrees>()
                                     .value());
  frc::SmartDashboard::PutNumber(
      table + "Steer Target Degrees",
      currentAngleSetpoint.convert<units::degrees>().value());
  frc::SmartDashboard::PutNumber(
      table + "Drive Velocity FPS",
      state.speed.convert<units::feet_per_second>().value());
  frc::SmartDashboard::PutNumber(
      table + "Drive Velocity Target FPS",
      currentDriveSetpoint.convert<units::feet_per_second>().value());
}

units::meter_t SwerveModule::ConvertMotorToWheelDistance(
    units::radian_t motorRotations) {
  return units::ConvertAngularDistanceToLinearDistance(
      motorRotations / constants::swerve::physical::DRIVE_GEARING,
      constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2);
}

units::meters_per_second_t SwerveModule::ConvertMotorSpeedToWheelVelocity(
    units::radians_per_second_t motorVelocity) {
  return units::ConvertAngularVelocityToLinearVelocity(
      motorVelocity / constants::swerve::physical::DRIVE_GEARING,
      constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2);
}

units::radian_t SwerveModule::ConvertWheelDistanceToMotorShaftRotations(
    units::meter_t wheelRotations) {
  return units::ConvertLinearDistanceToAngularDistance(
             wheelRotations,
             constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2) *
         constants::swerve::physical::DRIVE_GEARING;
}

units::radians_per_second_t SwerveModule::ConvertWheelVelocityToMotorVelocity(
    units::meters_per_second_t wheelVelocity) {
  return units::ConvertLinearVelocityToAngularVelocity(
             wheelVelocity,
             constants::swerve::physical::DRIVE_WHEEL_DIAMETER / 2) *
         constants::swerve::physical::DRIVE_GEARING;
}

units::radian_t SwerveModule::ConvertOutputShaftPositionToMotorShaftPosition(
    units::radian_t outputShaftPosition) {
  return outputShaftPosition * constants::swerve::physical::STEER_GEARING;
}

units::radians_per_second_t
SwerveModule::ConvertOutputShaftVelocityToMotorShaftVelocity(
    units::radians_per_second_t outputShaftVelocity) {
  return outputShaftVelocity * constants::swerve::physical::STEER_GEARING;
}
