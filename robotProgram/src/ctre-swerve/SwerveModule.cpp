// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "ctre-swerve/SwerveModule.h"
#include <ctre/phoenix6/configs/Configs.hpp>
#include "Constants.h"
#include "str/Units.h"

SwerveModule::SwerveModule(int driveMotorId, int steerMotorId, int steerEncId, double steerEncOffset, bool invertDrive, bool invertSteer) : driveMotor{driveMotorId, "*"},
                                                                                                                                            steerMotor{steerMotorId, "*"},
                                                                                                                                            steerEnc{steerEncId, "*"}
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
  driveSlotConfig.kV = constants::drivebase::gains::DRIVE_KV;
  driveSlotConfig.kA = constants::drivebase::gains::DRIVE_KA;
  driveSlotConfig.kS = constants::drivebase::gains::DRIVE_KS;
  driveSlotConfig.kP = constants::drivebase::gains::DRIVE_KP;
  driveSlotConfig.kI = constants::drivebase::gains::DRIVE_KI;
  driveSlotConfig.kD = constants::drivebase::gains::DRIVE_KD;
  driveConfig.Slot0 = driveSlotConfig;

  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.Feedback.SensorToMechanismRatio = constants::drivebase::physical::DRIVE_GEARING;
  driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants::drivebase::physical::SLIP_CURRENT.to<double>();
  driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants::drivebase::physical::SLIP_CURRENT.to<double>();
  driveConfig.MotorOutput.Inverted = invertDrive ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode status = driveMotor.GetConfigurator().Apply(driveConfig);
  if (!status.IsOK())
  {
    fmt::print("Swerve Module ConfigureDriveMotor() status wasn't ok it was: {}. More info: {}", status.GetName(), status.GetDescription());
  }
}

void SwerveModule::ConfigureSteerEncoder(double encOffset)
{
  ctre::phoenix6::configs::CANcoderConfiguration encoderConfig{};
  encoderConfig.MagnetSensor.MagnetOffset = encOffset;
  ctre::phoenix::StatusCode status = steerEnc.GetConfigurator().Apply(encoderConfig);
  if (!status.IsOK())
  {
    fmt::print("Swerve Module ConfigureSteerEncoder() status wasn't ok it was: {}. More info: {}", status.GetName(), status.GetDescription());
  }
}

void SwerveModule::ConfigureSteerMotor(bool invertSteer)
{
  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};

  ctre::phoenix6::configs::Slot0Configs steerSlotConfig{};
  steerSlotConfig.kV = constants::drivebase::gains::STEER_KV;
  steerSlotConfig.kA = constants::drivebase::gains::STEER_KA;
  steerSlotConfig.kS = constants::drivebase::gains::STEER_KS;
  steerSlotConfig.kP = constants::drivebase::gains::STEER_KP;
  steerSlotConfig.kI = constants::drivebase::gains::STEER_KI;
  steerSlotConfig.kD = constants::drivebase::gains::STEER_KD;
  steerConfig.Slot0 = steerSlotConfig;

  steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.Feedback.FeedbackRemoteSensorID = steerEnc.GetDeviceID();
  steerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::FusedCANcoder;
  steerConfig.Feedback.RotorToSensorRatio = constants::drivebase::physical::STEER_GEARING;
  steerConfig.MotorOutput.Inverted = invertSteer ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.MotionMagic.MotionMagicAcceleration = constants::drivebase::gains::STEER_MOTION_MAGIC_ACCEL;
  steerConfig.MotionMagic.MotionMagicCruiseVelocity = constants::drivebase::gains::STEER_MOTION_MAGIC_CRUISE_VEL;
  ctre::phoenix::StatusCode status = steerMotor.GetConfigurator().Apply(steerConfig);
  if (!status.IsOK())
  {
    fmt::print("Swerve Module ConfigureSteerMotor() status wasn't ok it was: {}. More info: {}", status.GetName(), status.GetDescription());
  }
}

frc::SwerveModulePosition SwerveModule::GetPosition(bool refresh)
{
  if (refresh)
  {
    ctre::phoenix::StatusCode status = ctre::phoenix6::BaseStatusSignal::WaitForAll(0_s, steerAngleSig, steerAngleVelSig, drivePositionSig, driveVelocitySig);
    if (!status.IsOK())
    {
      fmt::print("Swerve Module WaitForAll() status wasn't ok it was: {}. More info: {}", status.GetName(), status.GetDescription());
    }
  }

  units::turn_t driveTurns = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(drivePositionSig, driveVelocitySig);
  units::turn_t steerTurns = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(steerAngleSig, steerAngleVelSig);

  // This compensates for when the drive wheel rotates a little when you rotate the module
  driveTurns = driveTurns - (steerTurns * constants::drivebase::physical::DRIVE_STEER_COUPLING);

  internalState = frc::SwerveModulePosition{
      str::Units::ConvertAngularDistanceToLinearDistance(
          driveTurns,
          constants::drivebase::physical::WHEEL_DIAM / 2),
      frc::Rotation2d{
          steerTurns.convert<units::radians>()}};
  return internalState;
}

void SwerveModule::GoToState(frc::SwerveModuleState state, bool openLoop)
{
  frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(state, internalState.angle);

  // Reverses the expected module shimmy when rotating
  units::turns_per_second_t moduleTurnSpeed = steerAngleVelSig.GetValue();
  units::turns_per_second_t driveRateBackout = moduleTurnSpeed * constants::drivebase::physical::DRIVE_STEER_COUPLING;
  units::turns_per_second_t velocityToGoTo = str::Units::ConvertLinearVelocityToAngularVelocity(optimizedState.speed, constants::drivebase::physical::WHEEL_DIAM / 2) - driveRateBackout;

  steerMotor.SetControl(angleSetter.WithPosition(optimizedState.angle.Radians()));
  if (openLoop)
  {
    driveMotor.SetControl(
        voltageOpenLoopSetter.WithOutput(
            (velocityToGoTo / str::Units::ConvertLinearVelocityToAngularVelocity(
                                  constants::drivebase::physical::MAX_DRIVE_SPEED,
                                  constants::drivebase::physical::WHEEL_DIAM / 2)) *
            12.0_V));
  }
  else
  {
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
      str::Units::ConvertAngularVelocityToLinearVelocity(driveVelocitySig.GetValue(), constants::drivebase::physical::WHEEL_DIAM / 2),
      frc::Rotation2d{steerAngleSig.GetValue().convert<units::radians>()}};
}

void SwerveModule::ResetPosition()
{
  driveMotor.SetPosition(0_tr);
}

std::array<ctre::phoenix6::BaseStatusSignal *, 4> SwerveModule::GetSignals()
{
  return {&drivePositionSig, &driveVelocitySig, &steerAngleSig, &steerAngleVelSig};
}