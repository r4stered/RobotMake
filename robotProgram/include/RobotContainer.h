// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <functional>

#include "auto/Autos.h"
#include "str/Vision.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/DunkerSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/LedSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace str {
using DeadbandAndSquareFunc = std::function<double()>;
}  // namespace str

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  DrivebaseSubsystem& GetDrivebaseSubsystem();
  ShooterSubsystem& GetShooterSubsystem();
  DunkerSubsystem& GetDunkerSubsystem();
  IntakeSubsystem& GetIntakeSubsystem();
  Vision& GetVisionSystem();
  LedSubsystem& GetLedSubsystem();
  void CalculateNotePid();

  units::radian_t desiredAngle = 0_rad;

 private:
  void ConfigureBindings();
  frc2::CommandXboxController driverController{0};
  frc2::CommandXboxController operatorController{1};
  frc2::CommandXboxController testController{2};

  DrivebaseSubsystem driveSub;
  ShooterSubsystem shooterSub;
  IntakeSubsystem intakeSub;
  DunkerSubsystem dunkSub;
  ClimberSubsystem climbSub;
  LedSubsystem ledSub;

  Vision vision;

  autos::Autos autos{driveSub, shooterSub, intakeSub, dunkSub, vision};

  frc2::CommandPtr selfTestCmd = driveSub.SelfTest();
  frc2::CommandPtr measureWheelCmd = driveSub.MeasureWheelDiam([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneSteerCmd = driveSub.TuneSteerPID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });
  frc2::CommandPtr tuneDriveCmd = driveSub.TuneDrivePID([] {
    return frc::SmartDashboard::GetBoolean("Drivebase/DoneWithStep", false);
  });

  frc::PIDController notePid{.015, 0, 0};

  double rotSpeed = 0;

  frc2::CommandPtr resetPositionCmd =
      driveSub.ResetPosition([] { return frc::Pose2d{}; });
  frc2::CommandPtr tunePathPidCmd = driveSub.TunePathPid();
  frc2::CommandPtr donePathTuningCmd = driveSub.DoneTuningPathPids();
  frc2::CommandPtr zeroYawCMD = driveSub.ZeroYawCMD();

  frc2::CommandPtr rainbowLed = ledSub.SetBothToRainbow();

  frc2::CommandPtr SpinUpShooterBasedOnDist(
      std::function<units::meter_t()> distToGoal);
  frc2::CommandPtr SpinUpShooter();
  frc2::CommandPtr BabySpinUpShooter();
  frc2::CommandPtr NotUsingShooter();

  frc2::CommandPtr DunkNote();
  frc2::CommandPtr StopDunk();

  frc2::CommandPtr IntakeNote();

  frc2::CommandPtr RumbleDriver();
  frc2::CommandPtr RumbleOperator();

  std::function<frc2::CommandPtr()> GetAStarCmd();

  str::DeadbandAndSquareFunc DeadbandAndSquare(
      std::function<double()> joystickValue);

  double ShouldFlipControlsForDriver(double val);
  units::radian_t ShouldFlipAngleForDriver(units::radian_t targetAngle);
};
