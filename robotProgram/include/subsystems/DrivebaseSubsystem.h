// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SubsystemBase.h>

#include <functional>
#include <string>
#include <unordered_map>

#include <ctre/phoenix6/SignalLogger.hpp>

#include "Constants.h"
#include "choreo/lib/Choreo.h"
#include "choreo/lib/ChoreoTrajectory.h"
#include "frc/controller/PIDController.h"
#include "str/SwerveDrive.h"

class DrivebaseSubsystem : public frc2::SubsystemBase {
 public:
  DrivebaseSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  void UpdateOdometry();
  frc2::CommandPtr ResetPosition(std::function<frc::Pose2d()> newPosition);

  frc2::CommandPtr BabyDriveFactory(std::function<double()> fow,
                                    std::function<double()> side,
                                    std::function<double()> rot,
                                    std::function<bool()> fieldOriented);

  frc2::CommandPtr DriveFactory(std::function<double()> fow,
                                std::function<double()> side,
                                std::function<double()> rot,
                                std::function<bool()> fieldOriented);
  frc2::CommandPtr TurnToAngleFactory(
      std::function<double()> fow, std::function<double()> side,
      std::function<frc::TrapezoidProfile<units::radians>::State()>
          angleProfile,
      std::function<bool()> wantsToOverride, bool fieldRelative);

  frc2::CommandPtr SelfTest();
  frc2::CommandPtr MeasureWheelDiam(std::function<bool()> done);
  frc2::CommandPtr TuneSteerPID(std::function<bool()> done);
  frc2::CommandPtr TuneDrivePID(std::function<bool()> done);
  frc2::CommandPtr TunePathPid();
  frc2::CommandPtr DoneTuningPathPids();
  frc2::CommandPtr FollowChoreoTrajectory(
      std::function<std::string()> pathName);
  frc2::CommandPtr ZeroYawCMD();
  frc2::CommandPtr SysIdQuasistaticSteer(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamicSteer(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdQuasistaticDrive(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamicDrive(frc2::sysid::Direction direction);

  frc2::CommandPtr WheelRadFwd();
  frc2::CommandPtr WheelRadRev();

  units::meter_t CalcDistanceFromSpeaker();

  void SetTranslationPIDs(double p, double i, double d);
  void SetRotationPIDs(double p, double i, double d);
  void SetPathTuning(bool onOff);
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp);
  void AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                            units::second_t timestamp,
                            const Eigen::Vector3d& stdDevs);
  frc::Pose2d GetRobotPose();
  frc::Pose2d GetOdomPose();
  bool InSafeZone();
  frc::Pose2d CalculateClosestGoodShooterPoint();
  frc::Pose2d CalculateClosestSafeSpot();
  frc::Pose2d BestShooterPoint();
  frc2::CommandPtr PathfindToSafeSpot(std::function<frc::Pose2d()> poseToGoTo);
  frc2::CommandPtr GoToPose(std::function<frc::Pose2d()> poseToGoTo);
  frc2::CommandPtr MoveAlongArc(std::function<double()> joystick,
                                std::function<units::radian_t()> startAngle);

 private:
  str::SwerveDrive swerveDrive{};

  void LoadChoreoTrajectories();
  void SetupAutoBuilder();
  bool ShouldMirrorPath();

  frc::PIDController xTranslationController{
      constants::swerve::pathplanning::TRANSLATION_P,
      constants::swerve::pathplanning::TRANSLATION_I,
      constants::swerve::pathplanning::TRANSLATION_D};
  frc::PIDController yTranslationController{
      constants::swerve::pathplanning::TRANSLATION_P,
      constants::swerve::pathplanning::TRANSLATION_I,
      constants::swerve::pathplanning::TRANSLATION_D};
  frc::PIDController rotationController{
      constants::swerve::pathplanning::ROTATION_P,
      constants::swerve::pathplanning::ROTATION_I,
      constants::swerve::pathplanning::ROTATION_D};

  choreolib::ChoreoControllerFunction choreoController;

  frc::ProfiledPIDController<units::radians> thetaController{
      constants::swerve::pathplanning::ROTATION_P,
      constants::swerve::pathplanning::ROTATION_I,
      constants::swerve::pathplanning::ROTATION_D,
      constants::swerve::pathplanning::GLOBAL_THETA_CONTROLLER_CONSTRAINTS};

  std::unordered_map<std::string, choreolib::ChoreoTrajectory> pathMap;
  bool pathTuning{false};
  bool HavePIDsChanged(units::scalar_t transP, units::scalar_t transI,
                       units::scalar_t transD, units::scalar_t rotP,
                       units::scalar_t rotI, units::scalar_t rotD);

  units::radian_t thruAngle{0};
  units::radian_t minArcAngle{0};
  units::radian_t maxArcAngle{0};
  frc::Pose2d lastPoseInMoveToArc;

  frc2::sysid::SysIdRoutine sysIdRoutineSteer{
      frc2::sysid::Config{
          // using amps here (10 A / s) and 65_A step
          frc2::sysid::ramp_rate_t{10}, units::volt_t{65}, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) {
            swerveDrive.GetFLModuleForChar().SetSteerMotorToCurrent(
                voltsToSend);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            swerveDrive.GetFLModuleForChar().UpdateSteerSysIdLog(log);
          },
          this}};

  frc2::sysid::SysIdRoutine sysIdRoutineDrive{
      frc2::sysid::Config{
          // using amps here (10 A / s) and 65_A step
          frc2::sysid::ramp_rate_t{10}, units::volt_t{65}, std::nullopt,
          [this](frc::sysid::State state) {
            ctre::phoenix6::SignalLogger().WriteString(
                "state", frc::sysid::SysIdRoutineLog::StateEnumToString(state));
          }},
      frc2::sysid::Mechanism{
          [this](units::volt_t voltsToSend) {
            swerveDrive.SetAllModulesToCurrent(voltsToSend);
          },
          [this](frc::sysid::SysIdRoutineLog* log) {
            swerveDrive.GetFLModuleForChar().UpdateDriveSysIdLog(log);
          },
          this}};
};
