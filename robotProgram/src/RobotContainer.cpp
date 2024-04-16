// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  notePid.SetTolerance(2);
}

void RobotContainer::ConfigureBindings() {
  testController.RightBumper().WhileTrue(BabySpinUpShooter());
  testController.RightBumper().OnFalse(NotUsingShooter());

  driveSub.SetDefaultCommand(driveSub.BabyDriveFactory(
      DeadbandAndSquare([this] { return -testController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -testController.GetLeftX(); }),
      DeadbandAndSquare([this] { return -testController.GetRightX(); }),
      [] { return true; }));

  operatorController.X().WhileTrue(shooterSub.GoToSpeedCmd([this] {
    return frc::ApplyDeadband<double>(operatorController.GetLeftY(), 0.1);
  }));

  operatorController.RightTrigger().WhileTrue(intakeSub.SuckInNotes());
  operatorController.LeftTrigger().WhileTrue(IntakeNote());

  operatorController.Back().WhileTrue(intakeSub.SpitOutNotes());

  operatorController.Start().WhileTrue(
      climbSub.ManualControl([] { return 1; }, [] { return -1; }));

  operatorController.LeftBumper().OnTrue(dunkSub.PivotDunkNotesOut());

  operatorController.Y().WhileTrue(
      DunkNote().AlongWith(shooterSub.GoToVelocityCmd(
          [] { return constants::shooter::SHOOTER_DUNK_SPEED; },
          [] { return true; })));
  operatorController.Y().OnFalse(StopDunk().AlongWith(
      shooterSub.GoToVelocityCmd([] { return 0_rpm; }, [] { return true; })));

  operatorController.A().WhileTrue(SpinUpShooter());
  operatorController.A().OnFalse(NotUsingShooter());

  operatorController.B().WhileTrue(SpinUpShooterBasedOnDist(
      [this] { return driveSub.CalcDistanceFromSpeaker(); }));
  operatorController.B().OnFalse(NotUsingShooter());

  driverController.LeftBumper().WhileTrue(driveSub.DriveFactory(
      DeadbandAndSquare([this] {
        return ShouldFlipControlsForDriver(-driverController.GetLeftY());
      }),
      [this] { return -rotSpeed; }, DeadbandAndSquare([this] {
        return ShouldFlipControlsForDriver(-driverController.GetRightX());
      }),
      [] { return false; }));

  driverController.RightTrigger().WhileTrue(
      driveSub
          .GoToPose([this] {
            frc::Pose2d closestPoint = driveSub.BestShooterPoint();
            return closestPoint;
          })
          .AndThen(driveSub.MoveAlongArc(
              [this] {
                return frc::ApplyDeadband<double>(-driverController.GetLeftX(),
                                                  .1);
              },
              [this] {
                return driveSub.CalculateClosestGoodShooterPoint()
                    .Rotation()
                    .Radians();
              }))
          .AlongWith(SpinUpShooter()));

  driverController.LeftTrigger().WhileTrue(driveSub.GoToPose([this] {
    auto ally = frc::DriverStation::GetAlliance();
    if (ally.has_value()) {
      if (ally.value() == frc::DriverStation::Alliance::kRed) {
        return constants::swerve::automation::RED_AMP;
      }
    }
    return constants::swerve::automation::BLUE_AMP;
  }));

  driverController.Start().OnTrue(driveSub.ZeroYawCMD());

  driveSub.SetDefaultCommand(driveSub.DriveFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      DeadbandAndSquare([this] { return -driverController.GetRightX(); }),
      [] { return true; }));

  driverController.Y().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(180_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  driverController.X().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(-90_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  driverController.B().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(90_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  driverController.A().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        return frc::TrapezoidProfile<units::radians>::State{
            ShouldFlipAngleForDriver(0_deg), 0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  driverController.RightBumper().OnTrue(driveSub.TurnToAngleFactory(
      DeadbandAndSquare([this] { return -driverController.GetLeftY(); }),
      DeadbandAndSquare([this] { return -driverController.GetLeftX(); }),
      [this] {
        frc::Translation2d goal =
            constants::swerve::automation::BLUE_ALLIANCE_GOAL;
        auto ally = frc::DriverStation::GetAlliance();
        if (ally.has_value()) {
          if (ally.value() == frc::DriverStation::Alliance::kRed) {
            goal = constants::swerve::automation::RED_ALLIANCE_GOAL;
          }
        }
        frc::Pose2d pose = driveSub.GetRobotPose();
        frc::Rotation2d angle{
            units::math::atan2(goal.Y() - pose.Translation().Y(),
                               goal.X() - pose.Translation().X())};
        return frc::TrapezoidProfile<units::radians>::State{angle.Radians(),
                                                            0_deg_per_s};
      },
      [this] { return std::abs(driverController.GetRightX()) > 0.1; }, true));

  frc::SmartDashboard::PutBoolean("Drivebase/DoneWithStep", false);

  frc::SmartDashboard::PutData("Drivebase/SelfTestCmd", selfTestCmd.get());

  frc::SmartDashboard::PutData("Drivebase/MeasureWheelCmd",
                               measureWheelCmd.get());

  frc::SmartDashboard::PutData("Drivebase/TuneSteerCmd", tuneSteerCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ZeroYawCMD", zeroYawCMD.get());

  frc::SmartDashboard::PutData("Drivebase/TuneDriveCmd", tuneDriveCmd.get());

  frc::SmartDashboard::PutData("Drivebase/ResetPosition",
                               resetPositionCmd.get());

  frc::SmartDashboard::PutData("Drivebase/PathTuningCmd", tunePathPidCmd.get());

  frc::SmartDashboard::PutData("Drivebase/DonePathTuningCmd",
                               donePathTuningCmd.get());

  // testController.Back().WhileTrue(driveSub.WheelRadFwd());
  // testController.Start().WhileTrue(driveSub.WheelRadRev());

  // testController.Back().WhileTrue(
  //     driveSub.SysIdQuasistaticSteer(frc2::sysid::Direction::kForward));
  // testController.Start().WhileTrue(
  //     driveSub.SysIdQuasistaticSteer(frc2::sysid::Direction::kReverse));
  // testController.LeftBumper().WhileTrue(
  //     driveSub.SysIdDynamicSteer(frc2::sysid::Direction::kForward));
  // testController.RightBumper().WhileTrue(
  //     driveSub.SysIdDynamicSteer(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     driveSub.SysIdQuasistaticDrive(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     driveSub.SysIdQuasistaticDrive(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     driveSub.SysIdDynamicDrive(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     driveSub.SysIdDynamicDrive(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     shooterSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     shooterSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // testController.A().WhileTrue(
  //     dunkSub.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // testController.B().WhileTrue(
  //     dunkSub.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // testController.X().WhileTrue(
  //     dunkSub.SysIdDynamic(frc2::sysid::Direction::kForward));
  // testController.Y().WhileTrue(
  //     dunkSub.SysIdDynamic(frc2::sysid::Direction::kReverse));

  rainbowLed.Schedule();
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return autos.GetSelectedAutoCmd.get();
}

frc2::CommandPtr RobotContainer::SpinUpShooterBasedOnDist(
    std::function<units::meter_t()> distToGoal) {
  return frc2::cmd::Sequence(
      ledSub.SetBothToBlinkRed(), shooterSub.GoToSpeedBasedOnGoal(distToGoal),
      ledSub.SetBothToSolidGreen(),
      frc2::cmd::Parallel(RumbleDriver(), RumbleOperator()));
}

frc2::CommandPtr RobotContainer::SpinUpShooter() {
  return frc2::cmd::Sequence(
      frc2::cmd::Deadline(
          shooterSub.GoToVelocityCmd(
              [] { return constants::shooter::SHOOTER_SPEED; },
              [] { return false; }),
          ledSub.SetBothToTach(
              [this] {
                return shooterSub.GetLeftShooterCurrentVelocity().value();
              },
              [] { return constants::shooter::SHOOTER_SPEED.value(); })),
      ledSub.SetBothToSolidGreen(),
      frc2::cmd::Parallel(RumbleDriver(), RumbleOperator()));
}
frc2::CommandPtr RobotContainer::BabySpinUpShooter() {
  return frc2::cmd::Sequence(
      frc2::cmd::Deadline(
          shooterSub.GoToVelocityCmd(
              [] { return constants::shooter::SHOOTER_SPEED * .10; },
              [] { return false; }),
          ledSub.SetBothToTach(
              [this] {
                return shooterSub.GetLeftShooterCurrentVelocity().value();
              },
              [] { return constants::shooter::SHOOTER_SPEED.value() * .10; })),
      ledSub.SetBothToSolidGreen(),
      frc2::cmd::Parallel(RumbleDriver(), RumbleOperator()));
}

frc2::CommandPtr RobotContainer::NotUsingShooter() {
  return frc2::cmd::Sequence(ledSub.SetBothToOff(),
                             shooterSub.GoToSpeedCmd([] { return 0; }));
}

frc2::CommandPtr RobotContainer::IntakeNote() {
  return frc2::cmd::Sequence(
      ledSub.SetBothToBlinkOrange(), intakeSub.SuckInUntilNoteIsSeen(),
      ledSub.SetBothToSolidOrange(),
      frc2::cmd::Parallel(RumbleDriver(), RumbleOperator()));
}

frc2::CommandPtr RobotContainer::DunkNote() {
  return frc2::cmd::Sequence(dunkSub.PivotDunkNotesOut(),
                             dunkSub.DunkTheNotes());
}

frc2::CommandPtr RobotContainer::StopDunk() {
  return frc2::cmd::Sequence(dunkSub.StopDunking(), dunkSub.PivotDunkNotesIn());
}

frc2::CommandPtr RobotContainer::RumbleDriver() {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 1.0);
             }),
             frc2::cmd::Wait(.5_s), frc2::cmd::RunOnce([this] {
               driverController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                   0.0);
      });
}

frc2::CommandPtr RobotContainer::RumbleOperator() {
  return frc2::cmd::Sequence(
             frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 1.0);
             }),
             frc2::cmd::Wait(.5_s), frc2::cmd::RunOnce([this] {
               operatorController.SetRumble(
                   frc::GenericHID::RumbleType::kBothRumble, 0.0);
             }))
      .FinallyDo([this] {
        operatorController.SetRumble(frc::GenericHID::RumbleType::kBothRumble,
                                     0.0);
      });
}

units::radian_t RobotContainer::ShouldFlipAngleForDriver(
    units::radian_t targetAngle) {
  auto ally = frc::DriverStation::GetAlliance();
  if (ally.has_value()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return targetAngle + 180_deg;
    }
  }
  return targetAngle;
}

double RobotContainer::ShouldFlipControlsForDriver(double val) {
  auto ally = frc::DriverStation::GetAlliance();
  if (ally.has_value()) {
    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return val;
    }
  }
  return -val;
}

DrivebaseSubsystem& RobotContainer::GetDrivebaseSubsystem() {
  return driveSub;
}

ShooterSubsystem& RobotContainer::GetShooterSubsystem() {
  return shooterSub;
}

DunkerSubsystem& RobotContainer::GetDunkerSubsystem() {
  return dunkSub;
}

IntakeSubsystem& RobotContainer::GetIntakeSubsystem() {
  return intakeSub;
}

Vision& RobotContainer::GetVisionSystem() {
  return vision;
}

LedSubsystem& RobotContainer::GetLedSubsystem() {
  return ledSub;
}

str::DeadbandAndSquareFunc RobotContainer::DeadbandAndSquare(
    std::function<double()> joystickValue) {
  return [joystickValue]() {
    double deadband = frc::ApplyDeadband<double>(joystickValue(), 0.1);
    return std::abs(deadband) * deadband;
  };
}

std::function<frc2::CommandPtr()> RobotContainer::GetAStarCmd() {
  return [this] {
    return driveSub.PathfindToSafeSpot(
        [this] { return driveSub.CalculateClosestSafeSpot(); });
  };
}

void RobotContainer::CalculateNotePid() {
  auto result = vision.GetNoteCamResult();
  notePid.SetSetpoint(0);
  if (result.HasTargets()) {
    rotSpeed = notePid.Calculate(result.GetBestTarget().GetYaw());
  } else {
    rotSpeed = 0;
  }
}
