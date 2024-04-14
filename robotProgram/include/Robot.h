// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <optional>

#include "RobotContainer.h"

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
  frc2::Command* m_autonomousCommand;

  RobotContainer m_container;

  units::radian_t prevAngle;

  frc::PowerDistribution pdh{1, frc::PowerDistribution::ModuleType::kRev};
};
