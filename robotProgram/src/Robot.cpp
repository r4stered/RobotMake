// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <fmt/format.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandScheduler.h>

#include "str/DataUtils.h"

void Robot::RobotInit()
{
  DataUtils::SetupDataLogging();
  DataUtils::LogGitInfo();
}

void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { }

void Robot::DisabledExit() { }

void Robot::AutonomousInit()
{
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() { }

void Robot::AutonomousExit() { }

void Robot::TeleopInit()
{
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() { }

void Robot::TeleopExit() { }

void Robot::TestInit() { frc2::CommandScheduler::GetInstance().CancelAll(); }

void Robot::TestPeriodic() { }

void Robot::TestExit() { }

void Robot::SimulationPeriodic()
{
  // m_vision.SimPeriodic(m_container.GetDriveSub().GetState().pose);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
