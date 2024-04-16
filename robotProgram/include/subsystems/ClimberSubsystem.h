// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/trajectory/ExponentialProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

#include <functional>
#include <memory>

#include "Constants.h"

class ClimberSubsystem : public frc2::SubsystemBase {
 public:
  ClimberSubsystem();

  void ConfigureMotor();

  void Periodic() override;
  void SimulationPeriodic() override;
  bool AreBothAtHeight();
  void SetClimbHeight(units::meter_t newSetpoint);
  frc2::CommandPtr ManualControl(std::function<double()> left,
                                 std::function<double()> right);
  units::meter_t GetLeftClimberHeight();
  units::meter_t GetRightClimberHeight();

 private:
  rev::CANSparkMax leftClimberMotor{
      constants::climber::LEFT_CLIMBER_CAN_ID,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::CANSparkMax rightClimberMotor{
      constants::climber::RIGHT_CLIMBER_CAN_ID,
      rev::CANSparkLowLevel::MotorType::kBrushless};

  rev::SparkRelativeEncoder leftEncoder = leftClimberMotor.GetEncoder();

  rev::SparkRelativeEncoder rightEncoder = rightClimberMotor.GetEncoder();

  rev::SparkPIDController leftPIDController =
      leftClimberMotor.GetPIDController();
  rev::SparkPIDController rightPIDController =
      rightClimberMotor.GetPIDController();

  // these have to be uniqueptrs because for some reason the compiler refuses to
  // generate a copy constructor for ka, preventing normal reassignment
  std::unique_ptr<frc::ElevatorFeedforward> ffLeft =
      std::make_unique<frc::ElevatorFeedforward>(
          constants::climber::GAINS.kS, constants::climber::GAINS.kG,
          constants::climber::GAINS.kV, constants::climber::GAINS.kA);
  std::unique_ptr<frc::ElevatorFeedforward> ffRight =
      std::make_unique<frc::ElevatorFeedforward>(
          constants::climber::GAINS.kS, constants::climber::GAINS.kG,
          constants::climber::GAINS.kV, constants::climber::GAINS.kA);

  frc::ExponentialProfile<units::meters, units::volts> leftProfile{
      {10_V, constants::climber::GAINS.kV, constants::climber::GAINS.kA}};
  frc::ExponentialProfile<units::meters, units::volts> rightProfile{
      {10_V, constants::climber::GAINS.kV, constants::climber::GAINS.kA}};

  frc::ExponentialProfile<units::meters, units::volts>::State m_goal;

  frc::ExponentialProfile<units::meters, units::volts>::State leftSetpoint;
  frc::ExponentialProfile<units::meters, units::volts>::State rightSetpoint;

  units::volt_t ffResultLeft;
  units::volt_t ffResultRight;

  void InitSendable(wpi::SendableBuilder& builder) override;
  void SetGains(const constants::climber::ClimberGains newGains);
  constants::climber::ClimberGains GetGains();

  constants::climber::ClimberGains currentGains = constants::climber::GAINS;

  frc::sim::SimDeviceSim leftSparkSim{"SPARK MAX [22]"};
  frc::sim::SimDeviceSim rightSparkSim{"SPARK MAX [23]"};
  hal::SimDouble leftPos = leftSparkSim.GetDouble("Position");
  hal::SimDouble leftVel = leftSparkSim.GetDouble("Velocity");
  hal::SimDouble rightPos = rightSparkSim.GetDouble("Position");
  hal::SimDouble rightVel = rightSparkSim.GetDouble("Velocity");

  frc::sim::ElevatorSim leftElevatorSim{
      frc::DCMotor::NEO(1),
      constants::climber::CLIMBER_RATIO,
      constants::climber::CLIMBER_MASS,
      constants::climber::CLIMBER_SPOOL_RADIUS,
      0_m,
      16.5_in,
      true,
      0_m,
      {0.005}};

  frc::sim::ElevatorSim rightElevatorSim{
      frc::DCMotor::NEO(1),
      constants::climber::CLIMBER_RATIO,
      constants::climber::CLIMBER_MASS,
      constants::climber::CLIMBER_SPOOL_RADIUS,
      0_m,
      16.5_in,
      true,
      0_m,
      {0.005}};

  // Create a Mechanism2d display of an elevator
  frc::Mechanism2d m_mech2d{10_in / 1_m, 30_in / 1_m};
  frc::MechanismRoot2d* m_elevatorLeftRoot =
      m_mech2d.GetRoot("LeftRoot", 2_in / 1_m, 0.5_in / 1_m);
  frc::MechanismRoot2d* m_elevatorRightRoot =
      m_mech2d.GetRoot("RightRoot", 8_in / 1_m, 0.5_in / 1_m);
  frc::MechanismLigament2d* m_elevatorLeftMech2d =
      m_elevatorLeftRoot->Append<frc::MechanismLigament2d>(
          "ElevatorLeft", leftElevatorSim.GetPosition().value(), 90_deg);
  frc::MechanismLigament2d* m_elevatorRightMech2d =
      m_elevatorRightRoot->Append<frc::MechanismLigament2d>(
          "ElevatorRight", rightElevatorSim.GetPosition().value(), 90_deg);
};
