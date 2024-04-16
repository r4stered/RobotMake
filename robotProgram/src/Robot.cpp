// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <str/DataUtils.h>
#include <wpinet/PortForwarder.h>

void Robot::RobotInit() {
  str::DataUtils::SetupDataLogging();
  str::DataUtils::LogGitInfo();
  AddPeriodic([this] { m_container.GetDrivebaseSubsystem().UpdateOdometry(); },
              1 / 250_Hz);

  pdh.ClearStickyFaults();
  wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.54", 5800);
  wpi::PortForwarder::GetInstance().Add(5800, "10.20.53.55", 5800);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  // nt::NetworkTableInstance::GetDefault().Flush();

  m_container.CalculateNotePid();

  auto flvisionEst = m_container.GetVisionSystem().GetFLEstimatedGlobalPose();
  auto frvisionEst = m_container.GetVisionSystem().GetFREstimatedGlobalPose();
  auto blvisionEst = m_container.GetVisionSystem().GetBLEstimatedGlobalPose();
  auto brvisionEst = m_container.GetVisionSystem().GetBREstimatedGlobalPose();

  if (flvisionEst.has_value()) {
    auto est = flvisionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs =
        m_container.GetVisionSystem().GetEstimationStdDevs(estPose);
    m_container.GetDrivebaseSubsystem().AddVisionMeasurement(
        est.estimatedPose.ToPose2d(), est.timestamp, estStdDevs);
  }

  if (frvisionEst.has_value()) {
    auto est = frvisionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs =
        m_container.GetVisionSystem().GetEstimationStdDevs(estPose);
    m_container.GetDrivebaseSubsystem().AddVisionMeasurement(
        est.estimatedPose.ToPose2d(), est.timestamp, estStdDevs);
  }

  if (blvisionEst.has_value()) {
    auto est = blvisionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs =
        m_container.GetVisionSystem().GetEstimationStdDevs(estPose);
    m_container.GetDrivebaseSubsystem().AddVisionMeasurement(
        est.estimatedPose.ToPose2d(), est.timestamp, estStdDevs);
  }

  if (brvisionEst.has_value()) {
    auto est = brvisionEst.value();
    auto estPose = est.estimatedPose.ToPose2d();
    auto estStdDevs =
        m_container.GetVisionSystem().GetEstimationStdDevs(estPose);
    m_container.GetDrivebaseSubsystem().AddVisionMeasurement(
        est.estimatedPose.ToPose2d(), est.timestamp, estStdDevs);
  }

  frc::Pose2d robot2dPose = m_container.GetDrivebaseSubsystem().GetRobotPose();
  frc::Rotation3d robot3drot{0_rad, 0_rad, robot2dPose.Rotation().Radians()};
  frc::Pose3d robot3dPose{robot2dPose.X(), robot2dPose.Y(), 0_m, robot3drot};
  frc::Pose3d dunkerWheelsPose{constants::ascope::kdunkerWheelsOrigin};
  frc::Pose3d dunkerBottomArmPose{constants::ascope::kdunkerBottomArmOrigin};
  frc::Pose3d dunkerTopArmPose{constants::ascope::kdunkerTopArmOrigin};

  dunkerBottomArmPose = dunkerBottomArmPose.TransformBy(frc::Transform3d{
      frc::Translation3d{},
      frc::Rotation3d{
          0_rad, 0_rad,
          m_container.GetDunkerSubsystem().GetPivotAngle() - 58_deg}});
  dunkerTopArmPose = dunkerTopArmPose.TransformBy(frc::Transform3d{
      frc::Translation3d{},
      frc::Rotation3d{
          0_rad, 0_rad,
          m_container.GetDunkerSubsystem().GetPivotAngle() + 80_deg}});

  units::meter_t dunkerPoseX =
      dunkerBottomArmPose.Translation().X() -
      (11_in *
       units::math::cos(m_container.GetDunkerSubsystem().GetPivotAngle()));
  units::meter_t dunkerPoseY =
      dunkerBottomArmPose.Translation().Z() +
      (11_in *
       units::math::sin(m_container.GetDunkerSubsystem().GetPivotAngle()));

  dunkerWheelsPose =
      frc::Pose3d{frc::Translation3d{dunkerPoseX, 0_m, dunkerPoseY},
                  dunkerWheelsPose.Rotation()};

  std::array<double, 7> robotPoseArr = {
      robot3dPose.Translation().X().value(),
      robot3dPose.Translation().Y().value(),
      robot3dPose.Translation().Z().value(),
      robot3dPose.Rotation().GetQuaternion().W(),
      robot3dPose.Rotation().GetQuaternion().X(),
      robot3dPose.Rotation().GetQuaternion().Y(),
      robot3dPose.Rotation().GetQuaternion().Z()};

  std::array<double, 7> dunkerWheelsArr = {
      dunkerWheelsPose.Translation().X().value(),
      dunkerWheelsPose.Translation().Y().value(),
      dunkerWheelsPose.Translation().Z().value(),
      dunkerWheelsPose.Rotation().GetQuaternion().W(),
      dunkerWheelsPose.Rotation().GetQuaternion().X(),
      dunkerWheelsPose.Rotation().GetQuaternion().Y(),
      dunkerWheelsPose.Rotation().GetQuaternion().Z()};

  std::array<double, 7> dunkerBottomArmArr = {
      dunkerBottomArmPose.Translation().X().value(),
      dunkerBottomArmPose.Translation().Y().value(),
      dunkerBottomArmPose.Translation().Z().value(),
      dunkerBottomArmPose.Rotation().GetQuaternion().W(),
      dunkerBottomArmPose.Rotation().GetQuaternion().X(),
      dunkerBottomArmPose.Rotation().GetQuaternion().Y(),
      dunkerBottomArmPose.Rotation().GetQuaternion().Z()};

  std::array<double, 7> dunkerTopArmArr = {
      dunkerTopArmPose.Translation().X().value(),
      dunkerTopArmPose.Translation().Y().value(),
      dunkerTopArmPose.Translation().Z().value(),
      dunkerTopArmPose.Rotation().GetQuaternion().W(),
      dunkerTopArmPose.Rotation().GetQuaternion().X(),
      dunkerTopArmPose.Rotation().GetQuaternion().Y(),
      dunkerTopArmPose.Rotation().GetQuaternion().Z()};

  frc::SmartDashboard::PutNumberArray("ascope/robotPose3d", robotPoseArr);
  frc::SmartDashboard::PutNumberArray("ascope/dunkerWheelsPose",
                                      dunkerWheelsArr);
  frc::SmartDashboard::PutNumberArray("ascope/dunkerBottomArmPose",
                                      dunkerBottomArmArr);
  frc::SmartDashboard::PutNumberArray("ascope/dunkerTopArmPose",
                                      dunkerTopArmArr);
}

void Robot::SimulationPeriodic() {
  m_container.GetVisionSystem().SimPeriodic(
      m_container.GetDrivebaseSubsystem().GetOdomPose());
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {
  m_container.desiredAngle =
      m_container.GetDrivebaseSubsystem().GetRobotPose().Rotation().Radians();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
