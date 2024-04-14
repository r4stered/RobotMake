// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycle.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/simulation/DutyCycleSim.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismObject2d.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/CANSparkMax.h>

#include <functional>
#include <memory>

#include <ctre/phoenix6/TalonFX.hpp>

#include "Constants.h"

class DunkerSubsystem : public frc2::SubsystemBase {
 public:
  DunkerSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;
  units::radian_t GetPivotAngle();
  frc2::CommandPtr PivotDunkNotesOut();
  frc2::CommandPtr PivotDunkNotesIn();
  frc2::CommandPtr DunkManual(std::function<double()> speed);
  frc2::CommandPtr DunkTheNotes();
  frc2::CommandPtr StopDunking();
  frc2::CommandPtr JammedDunkNotes();
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  void ConfigureMotors();
  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::dunker::DunkerGains newGains);
  constants::dunker::DunkerGains GetGains();

  void SetDunkSpeed(double speed);
  void SetPivotGoal(units::radian_t angleGoal);
  bool IsPivotAtGoal();

  units::radian_t ConvertEncoderToAngle(double encoderReading);
  double ConvertAngleToEncoder(units::radian_t angle);

  rev::CANSparkMax dunkPivotMotor{constants::dunker::PIVOT_DUNKER_CAN_ID,
                                  rev::CANSparkLowLevel::MotorType::kBrushless};

  frc::DigitalInput pivotEncoderPort{constants::dunker::PIVOT_ENCODER_PORT};
  frc::DutyCycle pivotEncoder{pivotEncoderPort};

  std::unique_ptr<frc::ArmFeedforward> pivotFeedforward =
      std::make_unique<frc::ArmFeedforward>(
          constants::dunker::GAINS.kS, constants::dunker::GAINS.kG,
          constants::dunker::GAINS.kV, constants::dunker::GAINS.kA);
  frc::ProfiledPIDController<units::radians> pivotController{
      constants::dunker::GAINS.kP.value(), constants::dunker::GAINS.kI.value(),
      constants::dunker::GAINS.kD.value(),
      constants::dunker::PIVOT_CONTROLLER_CONSTRAINTS};

  ctre::phoenix6::hardware::TalonFX dunkMotor{constants::dunker::DUNKER_CAN_ID};
  ctre::phoenix6::controls::DutyCycleOut dutyController{0};

  units::radian_t currentPivotPos = 0_rad;
  constants::dunker::DunkerGains currentGains = constants::dunker::GAINS;

  // sim stuff
  frc::DCMotor pivotGearbox = frc::DCMotor::NEO(1);
  frc::sim::SingleJointedArmSim pivotSim{
      pivotGearbox,
      constants::dunker::PIVOT_GEAR_RATIO,
      frc::sim::SingleJointedArmSim::EstimateMOI(
          constants::dunker::PIVOT_ARM_LENGTH, constants::dunker::PIVOT_MASS),
      constants::dunker::PIVOT_ARM_LENGTH,
      constants::dunker::DUNKER_MIN_ANGLE,
      constants::dunker::DUNKER_MAX_ANGLE,
      true,
      constants::dunker::DUNKER_MIN_ANGLE};

  frc::sim::DutyCycleSim encoderSim{pivotEncoder};

  frc::Mechanism2d pivotMech{60, 60};
  frc::MechanismRoot2d* armPivotPoint = pivotMech.GetRoot("PivotRoot", 30, 30);
  frc::MechanismLigament2d* pivotArm =
      armPivotPoint->Append<frc::MechanismLigament2d>(
          "Pivot Arm", 30, -GetPivotAngle() - 180_deg, 6,
          frc::Color8Bit{frc::Color::kYellow});

  // characterization stuff

  units::radian_t prevPivotPosition = 0_rad;
  units::radians_per_second_t pivotVelocity = 0_rad_per_s;
  bool currentlyCharacterizing = false;

  frc2::sysid::SysIdRoutine sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                          std::nullopt},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) {
            dunkPivotMotor.SetVoltage(voltsToSend);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            log->Motor("dunk-pivot")
                .voltage(dunkPivotMotor.Get() *
                         frc::RobotController::GetBatteryVoltage())
                .position(GetPivotAngle().convert<units::turns>())
                .velocity(pivotVelocity.convert<units::turns_per_second>());
          },
          this},
  };
};
