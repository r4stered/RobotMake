// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/SwerveDrive.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/TimedRobot.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/FieldObject2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/math.h>

#include <cmath>
#include <fstream>
#include <iostream>

#include "Constants.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Twist2d.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc/kinematics/SwerveModulePosition.h"
#include "frc/kinematics/SwerveModuleState.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "frc2/command/Requirements.h"
#include "str/SwerveDriveSim.h"
#include "units/angle.h"

using namespace str;

SwerveDrive::SwerveDrive() {
  frc::SmartDashboard::PutData(&ntField);

  for (int i = 0; i < 4; i++) {
    const auto& moduleSignals = swerveModules[i].GetSignals();
    allModuleSignals[(i * 8) + 0] = moduleSignals[0];  // steer pos
    allModuleSignals[(i * 8) + 1] = moduleSignals[1];  // steer vel
    allModuleSignals[(i * 8) + 2] = moduleSignals[2];  // steer vol
    allModuleSignals[(i * 8) + 3] = moduleSignals[3];  // steer current
    allModuleSignals[(i * 8) + 4] = moduleSignals[4];  // drive pos
    allModuleSignals[(i * 8) + 5] = moduleSignals[5];  // drive vel
    allModuleSignals[(i * 8) + 6] = moduleSignals[6];  // drive vol
    allModuleSignals[(i * 8) + 7] = moduleSignals[7];  // drive current
  }
  allModuleSignals[allModuleSignals.size() - 2] = &imu.GetYaw();
  allModuleSignals[allModuleSignals.size() - 1] =
      &imu.GetAngularVelocityZWorld();

  for (const auto& signal : allModuleSignals) {
    ctre::phoenix::StatusCode status = signal->SetUpdateFrequency(250_Hz);
    if (!status.IsOK()) {
      frc::DataLogManager::Log(fmt::format(
          "Signal {} was unable to set its frequence to 250 Hz! "
          "Error: {}, More Info: {}",
          signal->GetName(), status.GetName(), status.GetDescription()));
    }
  }

  imu.OptimizeBusUtilization();
  for (int i = 0; i < 4; i++) {
    swerveModules[i].OptimizeBusSignals();
  }

  ctre::phoenix6::configs::Pigeon2Configuration imuConfig;
  imuConfig.MountPose.MountPosePitch = -0.4168;
  imuConfig.MountPose.MountPoseRoll = -89.879;
  imuConfig.MountPose.MountPoseYaw = 89.9885;
  imu.GetConfigurator().Apply(imuConfig);

  imu.SetYaw(0_rad);
}

void SwerveDrive::Drive(units::meters_per_second_t vx,
                        units::meters_per_second_t vy,
                        units::radians_per_second_t omega, bool openLoop,
                        bool fieldOriented) {
  // offset if red
  auto allyValue = frc::DriverStation::GetAlliance();
  frc::Rotation2d rotationOffset = 0_rad;
  if (allyValue) {
    if (allyValue.value() == frc::DriverStation::Alliance::kRed &&
        fieldOriented) {
      rotationOffset = frc::Rotation2d{units::radian_t{std::numbers::pi}};
    }
  }
  if (fieldOriented) {
    frc::ChassisSpeeds newChassisSpeeds =
        frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            vx, vy, omega, GetHeading() + rotationOffset);
    SetChassisSpeeds(newChassisSpeeds, openLoop);
  } else {
    frc::ChassisSpeeds robotSpeeds;
    robotSpeeds.vx = vx;
    robotSpeeds.vy = vy;
    robotSpeeds.omega = omega;
    SetChassisSpeeds(robotSpeeds, openLoop);
  }
}

void SwerveDrive::TareEverything() {
  for (int i = 0; i < 4; i++) {
    swerveModules[i].ResetPosition();
    modulePositions[i] = swerveModules[i].GetPosition(true);
  }
  poseEstimator.ResetPosition(imu.GetRotation2d(), modulePositions,
                              frc::Pose2d{});
  odom.ResetPosition(imu.GetRotation2d(), modulePositions, frc::Pose2d{});
}

void SwerveDrive::SeedFieldRelative() {
  // fieldRelativeOffset = GetPose().Rotation().Radians();
  fmt::print("field relative offset {}\n", fieldRelativeOffset.value());
}

void SwerveDrive::SeedFieldRelative(const frc::Pose2d& location) {
  // fieldRelativeOffset = location.Rotation().Radians();
  poseEstimator.ResetPosition(location.Rotation(), modulePositions, location);
  odom.ResetPosition(location.Rotation(), modulePositions, location);
}

void SwerveDrive::SetChassisSpeeds(const frc::ChassisSpeeds& newChassisSpeeds,
                                   bool openLoop) {
  SetModuleStates(constants::swerve::physical::KINEMATICS.ToSwerveModuleStates(
                      frc::ChassisSpeeds::Discretize(newChassisSpeeds, 0.02_s)),
                  openLoop);
}

void SwerveDrive::SetModuleStates(
    const std::array<frc::SwerveModuleState, 4>& desiredStates, bool openLoop,
    bool optimize) {
  units::meters_per_second_t maxSpeed;
  if (openLoop) {
    maxSpeed = constants::swerve::physical::MAX_LINEAR_SPEED;
  } else {
    maxSpeed = constants::swerve::physical::MAX_LINEAR_SPEED_FOC;
  }

  std::array<frc::SwerveModuleState, 4> desaturatedStates = desiredStates;
  frc::SwerveDriveKinematics<4>::DesaturateWheelSpeeds(
      static_cast<wpi::array<frc::SwerveModuleState, 4>*>(&desaturatedStates),
      maxSpeed);

  std::array<double, 8> advantageScopeSwerveView{
      desaturatedStates[0].angle.Degrees().value(),
      desaturatedStates[0].speed.value(),
      desaturatedStates[1].angle.Degrees().value(),
      desaturatedStates[1].speed.value(),
      desaturatedStates[2].angle.Degrees().value(),
      desaturatedStates[2].speed.value(),
      desaturatedStates[3].angle.Degrees().value(),
      desaturatedStates[3].speed.value()};
  frc::SmartDashboard::PutNumberArray("Drivebase/SwerveDesiredAScope",
                                      advantageScopeSwerveView);

  for (size_t i = 0; i < swerveModules.size(); i++) {
    swerveModules[i].GoToState(desaturatedStates[i], openLoop, optimize);
  }
}

frc::Rotation2d SwerveDrive::GetHeading() const {
  if (frc::DriverStation::IsTeleop()) {
    return GetGyroYaw();
  } else {
    return GetPose().Rotation();
  }
}

frc::Rotation2d SwerveDrive::GetGyroYaw() const {
  return frc::Rotation2d{imuYaw};
}

frc::Field2d& SwerveDrive::GetField() {
  return ntField;
}

void SwerveDrive::Log() {
  frc::SmartDashboard::PutNumber("Drivebase/Averge Odom Frequency",
                                 (1 / averageLoopTime).value());

  ntField.GetObject("Estimated Robot Pose")->SetPose(GetPose());
  ntField.GetObject("Estimated Robot Modules")->SetPoses(GetModulePoses());
  ntField.GetObject("Odom Pose")->SetPose(GetOdomPose());

  std::array<frc::SwerveModuleState, 4> moduleStates;

  for (size_t i = 0; i < swerveModules.size(); i++) {
    moduleStates[i] = swerveModules[i].GetState();
    swerveModules[i].Log(i);
  }

  std::array<double, 8> advantageScopeSwerveView{
      moduleStates[0].angle.Degrees().value(), moduleStates[0].speed.value(),
      moduleStates[1].angle.Degrees().value(), moduleStates[1].speed.value(),
      moduleStates[2].angle.Degrees().value(), moduleStates[2].speed.value(),
      moduleStates[3].angle.Degrees().value(), moduleStates[3].speed.value()};
  frc::SmartDashboard::PutNumberArray("Drivebase/SwerveCurrentAScope",
                                      advantageScopeSwerveView);
}

void SwerveDrive::SimulationUpdate() {
  std::array<units::volt_t, 4> driveInputs;
  std::array<units::volt_t, 4> steerInputs;
  for (int i = 0; i < 4; i++) {
    steerInputs[i] =
        units::volt_t{allModuleSignals[(i * 8) + 2]->GetValueAsDouble()};
    driveInputs[i] =
        units::volt_t{allModuleSignals[(i * 8) + 6]->GetValueAsDouble()};
  }

  swerveSim.SetDriveInputs(driveInputs);
  swerveSim.SetSteerInputs(steerInputs);

  units::second_t now = frc::Timer::GetFPGATimestamp();
  swerveSim.Update(now - lastUpdateTimestamp);
  lastUpdateTimestamp = now;

  totalCurrentDraw =
      swerveSim.GetSteerCurrentDraw() + swerveSim.GetDriveCurrentDraw();

  std::array<SimState, 4> state = swerveSim.GetState();
  std::array<frc::SwerveModulePosition, 4> positions{};
  for (size_t i = 0; i < swerveModules.size(); i++) {
    swerveModules[i].SimulationUpdate(state[i].drivePos, state[i].driveVel,
                                      state[i].steerPos, state[i].steerVel);
    frc::SwerveModulePosition currentPos = swerveModules[i].GetCachedPosition();
    positions[i] = frc::SwerveModulePosition{
        currentPos.distance - lastPositions[i].distance, currentPos.angle};
    lastPositions[i].distance = currentPos.distance;
  }

  frc::Twist2d change =
      constants::swerve::physical::KINEMATICS.ToTwist2d(positions);
  lastAngle = lastAngle + frc::Rotation2d{change.dtheta};
  imuSimState.SetRawYaw(lastAngle.Degrees());
}

frc::Pose2d SwerveDrive::GetPose() const {
  return poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SwerveDrive::GetOdomPose() const {
  return odom.GetPose();
}

units::ampere_t SwerveDrive::GetCurrentDraw() const {
  return totalCurrentDraw;
}

std::array<frc::Pose2d, 4> SwerveDrive::GetModulePoses() const {
  std::array<frc::Pose2d, 4> poses;
  for (int i = 0; i < 4; i++) {
    poses[i] = GetPose().TransformBy(
        frc::Transform2d{constants::swerve::physical::moduleLocations[i],
                         swerveModules[i].GetCachedPosition().angle});
  }
  return poses;
}

void SwerveDrive::UpdateOdometry() {
  lastTime = currentTime;
  currentTime = units::second_t{ctre::phoenix6::GetCurrentTimeSeconds()};
  averageLoopTime = currentTime - lastTime;

  ctre::phoenix::StatusCode status =
      ctre::phoenix6::BaseStatusSignal::WaitForAll(2.0 / 250_Hz,
                                                   allModuleSignals);

  if (frc::RobotBase::IsReal()) {
    if (!status.IsOK()) {
      frc::DataLogManager::Log(
          fmt::format("UpdateOdometry failed: {}. More info: {}",
                      status.GetName(), status.GetDescription()));
    }
  }

  for (int i = 0; i < 4; i++) {
    modulePositions[i] = swerveModules[i].GetPosition(false);
  }

  imuYaw = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      imu.GetYaw(), imu.GetAngularVelocityZWorld());
  imuRate = imu.GetAngularVelocityZWorld().GetValue();
  poseEstimator.Update(frc::Rotation2d{imuYaw + fieldRelativeOffset},
                       modulePositions);
  odom.Update(frc::Rotation2d{imuYaw + fieldRelativeOffset}, modulePositions);
}

frc2::CommandPtr SwerveDrive::SelfTest(frc2::Requirements reqs) {
  return frc2::cmd::Sequence(
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState forwardState{0_mps, frc::Rotation2d{0_deg}};
            SetModuleStates(
                {forwardState, forwardState, forwardState, forwardState}, false,
                false);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState leftState{0_mps, frc::Rotation2d{90_deg}};
            SetModuleStates({leftState, leftState, leftState, leftState}, false,
                            false);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState backState{0_mps, frc::Rotation2d{180_deg}};
            SetModuleStates({backState, backState, backState, backState}, false,
                            false);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState rightState{0_mps, frc::Rotation2d{270_deg}};
            SetModuleStates({rightState, rightState, rightState, rightState},
                            false, false);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState forwardState{0_mps, frc::Rotation2d{0_deg}};
            SetModuleStates(
                {forwardState, forwardState, forwardState, forwardState}, false,
                false);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState goForward{3_fps, frc::Rotation2d{0_deg}};
            SetModuleStates({goForward, goForward, goForward, goForward}, false,
                            true);
          },
          reqs)
          .WithTimeout(2_s),
      frc2::cmd::Run(
          [this] {
            frc::SwerveModuleState goBack{-3_fps, frc::Rotation2d{0_deg}};
            SetModuleStates({goBack, goBack, goBack, goBack}, false, true);
          },
          reqs)
          .WithTimeout(2_s));
}

frc2::CommandPtr SwerveDrive::MeasureWheelDiam(std::function<bool()> done,
                                               frc2::Requirements reqs) {
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [this] {
            for (int i = 0; i < 4; i++) {
              swerveModules[i].PushMode(true);
            }
            std::cout << "Please push the drivebase forward exactly one "
                         "foot!\n\n\n\n";
          },
          reqs),
      frc2::cmd::Race(frc2::cmd::WaitUntil(done),
                      frc2::cmd::Run(
                          [this] {
                            for (int i = 0; i < 4; i++) {
                              swerveModules[i].LockSteerAtZero();
                            }
                          },
                          reqs)),
      frc2::cmd::RunOnce(
          [this] {
            for (int i = 0; i < 4; i++) {
              units::radian_t motorRotations =
                  swerveModules[i].GetMotorRotations();
              units::radian_t outputShaftRotations =
                  motorRotations / constants::swerve::physical::DRIVE_GEARING;
              units::meter_t calculatedWheelRadius =
                  1_ft / (outputShaftRotations / 1_rad);
              std::cout << fmt::format(
                  "Drive motor rotated {} times \n Output shaft rotated {} "
                  "times. \n Your calculated wheel radius is {} "
                  "meters!\n\n\n\n",
                  motorRotations.value(), outputShaftRotations.value(),
                  calculatedWheelRadius.value());
            }
          },
          reqs),
      frc2::cmd::RunOnce(
          [this] {
            for (int i = 0; i < 4; i++) {
              swerveModules[i].PushMode(false);
            }
          },
          reqs));
}

frc2::CommandPtr SwerveDrive::TuneSteerPID(std::function<bool()> done,
                                           frc2::Requirements reqs) {
  std::string tablePrefix = "Drivebase/steerGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix, this] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", constants::swerve::steerGains.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", constants::swerve::steerGains.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", constants::swerve::steerGains.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", constants::swerve::steerGains.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", constants::swerve::steerGains.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", constants::swerve::steerGains.kD.value());
            for (int i = 0; i < 4; i++) {
              swerveModules[i].GoToState(
                  frc::SwerveModuleState{units::feet_per_second_t{0},
                                         frc::Rotation2d{0_rad}},
                  true, false);
            }
          },
          reqs),
      frc2::cmd::Run(
          [this, tablePrefix] {
            constants::swerve::ModuleSteerGains newGains{
                units::unit_t<
                    frc::SimpleMotorFeedforward<units::radians>::ka_unit>{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                units::unit_t<
                    frc::SimpleMotorFeedforward<units::radians>::kv_unit>{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::volt_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)},
            };

            if (newGains != swerveModules[0].GetCurrentSteerGains()) {
              for (int i = 0; i < 4; i++) {
                swerveModules[i].SetModuleSteerGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              swerveModules[i].GoToState(
                  frc::SwerveModuleState{
                      0_mps, frc::Rotation2d{units::degree_t{
                                 frc::SmartDashboard::GetNumber(
                                     tablePrefix + "setpoint", 0)}}},
                  true, false);
            }
          },
          reqs)
          .Until(done));
}

frc2::CommandPtr SwerveDrive::TuneDrivePID(std::function<bool()> done,
                                           frc2::Requirements reqs) {
  std::string tablePrefix = "Drivebase/driveGains/";
  return frc2::cmd::Sequence(
      frc2::cmd::RunOnce(
          [tablePrefix] {
            frc::SmartDashboard::PutNumber(tablePrefix + "setpoint", 0);
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kA", constants::swerve::driveGains.kA.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kV", constants::swerve::driveGains.kV.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kS", constants::swerve::driveGains.kS.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kP", constants::swerve::driveGains.kP.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kI", constants::swerve::driveGains.kI.value());
            frc::SmartDashboard::PutNumber(
                tablePrefix + "kD", constants::swerve::driveGains.kD.value());
          },
          reqs),
      frc2::cmd::Run(
          [this, tablePrefix] {
            constants::swerve::ModuleDriveGains newGains{
                units::unit_t<
                    frc::SimpleMotorFeedforward<units::meters>::ka_unit>{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kA", 0)},
                units::unit_t<
                    frc::SimpleMotorFeedforward<units::meters>::kv_unit>{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kV", 0)},
                units::volt_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kS", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kP", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kI", 0)},
                units::scalar_t{
                    frc::SmartDashboard::GetNumber(tablePrefix + "kD", 0)},
            };

            if (newGains != swerveModules[0].GetCurrentDriveGains()) {
              for (int i = 0; i < 4; i++) {
                swerveModules[i].SetModuleDriveGains(newGains);
              }
            }

            for (int i = 0; i < 4; i++) {
              swerveModules[i].GoToState(
                  frc::SwerveModuleState{
                      units::feet_per_second_t{frc::SmartDashboard::GetNumber(
                          tablePrefix + "setpoint", 0)},
                      frc::Rotation2d{0_rad}},
                  false, false);
            }
          },
          reqs)
          .Until(done));
}

void SwerveDrive::ZeroYaw() {
  units::radian_t targetAngle = 0_rad;
  auto ally = frc::DriverStation::GetAlliance();
  if (ally.has_value()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      targetAngle = 0_deg;
    }
    if (ally.value() == frc::DriverStation::Alliance::kBlue) {
      targetAngle = 180_deg;
    }
  }
  imu.SetYaw(targetAngle);
}

void SwerveDrive::SetGyroYaw(units::radian_t newYaw) {
  imu.SetYaw(newYaw);
}

void SwerveDrive::SetAllModulesToCurrent(units::volt_t voltsToSend) {
  swerveModules[0].LockSteerAtZero();
  swerveModules[1].LockSteerAtZero();
  swerveModules[2].LockSteerAtZero();
  swerveModules[3].LockSteerAtZero();
  swerveModules[0].SetDriveMotorToCurrent(voltsToSend);
  swerveModules[1].SetDriveMotorToCurrent(voltsToSend);
  swerveModules[2].SetDriveMotorToCurrent(voltsToSend);
  swerveModules[3].SetDriveMotorToCurrent(voltsToSend);
}

void SwerveDrive::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                       units::second_t timestamp) {
  // outside field, so we dont want to add this measurement to estimator,
  // because we know its wrong
  if (visionMeasurement.X() < 0_m || visionMeasurement.Y() < 0_m) {
    return;
  }
  if (visionMeasurement.X() > layout.GetFieldLength() ||
      visionMeasurement.Y() > layout.GetFieldWidth()) {
    return;
  }
  poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp);
}

void SwerveDrive::AddVisionMeasurement(const frc::Pose2d& visionMeasurement,
                                       units::second_t timestamp,
                                       const Eigen::Vector3d& stdDevs) {
  // outside field, so we dont want to add this measurement to estimator,
  // because we know its wrong
  if (visionMeasurement.X() < 0_m || visionMeasurement.Y() < 0_m) {
    return;
  }
  if (visionMeasurement.X() > layout.GetFieldLength() ||
      visionMeasurement.Y() > layout.GetFieldWidth()) {
    return;
  }
  wpi::array<double, 3> newStdDevs{stdDevs(0), stdDevs(1), stdDevs(2)};
  poseEstimator.AddVisionMeasurement(visionMeasurement, timestamp, newStdDevs);
}

frc2::CommandPtr SwerveDrive::WheelRadiusCmd(frc2::Requirements reqs,
                                             double direction) {
  return frc2::cmd::RunOnce(
             [this] {
               lastGyroYaw = imuYaw;
               accumGyroYaw = 0_rad;
               for (int i = 0; i < 4; i++) {
                 startWheelPositions[i] =
                     swerveModules[i].GetMotorRotations() /
                     constants::swerve::physical::DRIVE_GEARING;
               }
               omegaLimiter.Reset(0_rad_per_s);
               effectiveWheelRadius = 0_in;
             },
             reqs)
      .AndThen(frc2::cmd::RunEnd(
          [this, direction] {
            Drive(0_mps, 0_mps, omegaLimiter.Calculate(1_rad_per_s * direction),
                  true, false);
            accumGyroYaw += frc::AngleModulus(imuYaw - lastGyroYaw);
            lastGyroYaw = imuYaw;
            units::radian_t avgWheelPos = 0.0_rad;
            std::array<units::radian_t, 4> currentPositions;
            for (int i = 0; i < 4; i++) {
              currentPositions[i] = swerveModules[i].GetMotorRotations() /
                                    constants::swerve::physical::DRIVE_GEARING;
            }
            for (int i = 0; i < 4; i++) {
              avgWheelPos += units::math::abs(currentPositions[i] -
                                              startWheelPositions[i]);
            }
            avgWheelPos /= 4.0;
            effectiveWheelRadius = (accumGyroYaw * driveRadius) / avgWheelPos;
          },
          [this] {
            Drive(0_mps, 0_mps, 0_rad_per_s, true, false);
            fmt::print("WHEEL RADIUS: {}\n",
                       effectiveWheelRadius.convert<units::inches>().value());
          },
          reqs));
}
