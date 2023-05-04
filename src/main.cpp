// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <AHRS.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPoint.h>
#include <photonlib/PhotonUtils.h>
#include <units/length.h>

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with arcade steering.
 */
class Robot : public frc::TimedRobot
{
  frc::PWMSparkMax m_leftMotor{0};
  frc::PWMSparkMax m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::Joystick m_stick{0};
  AHRS navx{frc::SerialPort::kMXP};

public:
  void RobotInit() override
  {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);
    navx.Calibrate();
    navx.ZeroYaw();

    pathplanner::PathPlannerTrajectory traj1 = pathplanner::PathPlanner::generatePath(
        pathplanner::PathConstraints(4_mps, 3_mps_sq),
        pathplanner::PathPoint(frc::Translation2d(1_m, 1_m), frc::Rotation2d(0_deg)), // position, heading
        pathplanner::PathPoint(frc::Translation2d(3_m, 3_m), frc::Rotation2d(45_deg)) // position, heading
    );

    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(2_m, 5_m, 1_deg, units::degree_t{10});
    fmt::print("Range: {}\n", range.value());
  }

  void RobotPeriodic() override
  {
    fmt::print("Navx Yaw: {}\n", navx.GetYaw());
  }

  void TeleopPeriodic() override
  {
    // Drive with arcade style
    m_robotDrive.ArcadeDrive(-m_stick.GetY(), -m_stick.GetX());
  }
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif