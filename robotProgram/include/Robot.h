// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>

#include "RobotContainer.h"
// #include "Vision.h"
#include "str/Alert.h"
#include "str/SwerveModule.h"

class Robot : public frc::TimedRobot {
public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;
  void SimulationPeriodic() override;

private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  alert::Alert canError{"CAN errors detected, robot may not be controllable.",
    alert::AlertType::CRITICAL};
  RobotContainer m_container;
  // Vision m_vision;
};
