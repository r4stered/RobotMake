// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/PrintCommand.h"
#include "str/Vision.h"
#include "subsystems/DrivebaseSubsystem.h"
#include "subsystems/DunkerSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

namespace autos {

enum CommandSelector {
  DO_NOTHING,
  SQUARE,
  CHOREO_TEST,
  AMP_SIDE,
  MIDDLE_OF_DRIVER_STATION,
  THREE_NOTE_SOURCE,
  MIDDLE_SUB,
  AMP_SIDE_SINGLE,
  SIX_NOTE,
  PODIUM_NOTE,
  RUIN_UR_DAY,
  THREE_MIDDLE_SUB
};

class Autos {
 public:
  explicit Autos(DrivebaseSubsystem& driveSub, ShooterSubsystem& shooterSub,
                 IntakeSubsystem& intakeSub, DunkerSubsystem& dunkSub,
                 Vision& vision)
      : m_driveSub(driveSub),
        m_shooterSub(shooterSub),
        m_intakeSub(intakeSub),
        m_dunkerSub(dunkSub),
        m_vision(vision) {
    pathplanner::NamedCommands::registerCommand(
        "TestCommandPrint",
        frc2::PrintCommand("Test Print from PP Command").ToPtr());

    pathplanner::NamedCommands::registerCommand(
        "SpinUpShooter",
        shooterSub
            .GoToVelocityCmd([] { return 5200_rpm; }, [] { return false; })
            .WithTimeout(.75_s));

    pathplanner::NamedCommands::registerCommand(
        "StopShooter",
        shooterSub.GoToSpeedCmd([] { return 0; }).WithTimeout(.25_s));

    pathplanner::NamedCommands::registerCommand(
        "IntakeNote", intakeSub.SuckInUntilNoteIsSeen());

    pathplanner::NamedCommands::registerCommand(
        "SpitNote", intakeSub.SpitOutNotes().WithTimeout(.5_s));

    pathplanner::NamedCommands::registerCommand(
        "FeedNote", intakeSub.SuckInNotes().WithTimeout(1_s));

    pathplanner::NamedCommands::registerCommand(
        "ReadyToDunk",
        frc2::cmd::Parallel(
            frc2::cmd::Sequence(dunkSub.PivotDunkNotesOut(),
                                dunkSub.DunkManual([] { return 1; })),
            shooterSub.GoToVelocityCmd([] { return 2000_rpm; },
                                       [] { return true; }))
            .WithTimeout(2_s));

    pathplanner::NamedCommands::registerCommand(
        "ResetDunker",
        shooterSub.GoToVelocityCmd([] { return 0_rpm; }, [] { return true; })
            .AlongWith(
                dunkSub.StopDunking().AndThen(dunkSub.PivotDunkNotesIn())));

    pathplanner::NamedCommands::registerCommand(
        "GoToShooterPoint", driveSub.GoToPose([this] {
          frc::Pose2d closestPoint = m_driveSub.BestShooterPoint();
          return closestPoint;
        }));

    pathplanner::NamedCommands::registerCommand(
        "PointAtGoal",
        driveSub
            .TurnToAngleFactory(
                [] { return 0; }, [] { return 0; },
                [this] {
                  frc::Translation2d goal =
                      constants::swerve::automation::BLUE_ALLIANCE_GOAL;
                  auto ally = frc::DriverStation::GetAlliance();
                  if (ally.has_value()) {
                    if (ally.value() == frc::DriverStation::Alliance::kRed) {
                      goal = constants::swerve::automation::RED_ALLIANCE_GOAL;
                    }
                  }
                  frc::Pose2d pose = m_driveSub.GetRobotPose();
                  frc::Rotation2d angle{
                      units::math::atan2(goal.Y() - pose.Translation().Y(),
                                         goal.X() - pose.Translation().X())};
                  return frc::TrapezoidProfile<units::radians>::State{
                      angle.Radians(), 0_deg_per_s};
                },
                [] { return false; }, true)
            .WithTimeout(1_s));

    GetSelectedAutoCmd = frc2::cmd::Select<CommandSelector>(
        [this] { return chooser.GetSelected(); },
        std::pair{DO_NOTHING,
                  frc2::cmd::Print("ERROR: DO NOTHING AUTO SELECTED! YOU "
                                   "PROBABLY DIDNT MEAN THIS")},
        std::pair{SQUARE, pathplanner::PathPlannerAuto{"Square"}.ToPtr()},
        std::pair{CHOREO_TEST,
                  pathplanner::PathPlannerAuto{"ChoreoTestPath"}.ToPtr()},
        std::pair{AMP_SIDE, pathplanner::PathPlannerAuto{"AmpSide"}.ToPtr()},
        std::pair{
            MIDDLE_OF_DRIVER_STATION,
            pathplanner::PathPlannerAuto{"MiddleOfDriverStation"}.ToPtr()},
        std::pair{THREE_NOTE_SOURCE,
                  pathplanner::PathPlannerAuto{"ThreeNoteSource"}.ToPtr()},
        std::pair{MIDDLE_SUB,
                  pathplanner::PathPlannerAuto{"MiddleSub"}.ToPtr()},
        std::pair{THREE_MIDDLE_SUB,
                  pathplanner::PathPlannerAuto{"ThreeMiddleSub"}.ToPtr()},
        std::pair{AMP_SIDE_SINGLE,
                  pathplanner::PathPlannerAuto("AmpSideSingle").ToPtr()},
        std::pair{SIX_NOTE, pathplanner::PathPlannerAuto("SixNote").ToPtr()},
        std::pair{PODIUM_NOTE,
                  pathplanner::PathPlannerAuto("PodiumNote").ToPtr()},
        std::pair{RUIN_UR_DAY,
                  pathplanner::PathPlannerAuto("RuinUrDay").ToPtr()});

    chooser.SetDefaultOption("Do Nothing", CommandSelector::DO_NOTHING);
    chooser.AddOption("Drive in Square", CommandSelector::SQUARE);
    chooser.AddOption("Choreo Test", CommandSelector::CHOREO_TEST);
    chooser.AddOption("Amp Side", CommandSelector::AMP_SIDE);
    chooser.AddOption("Middle of Driver Station",
                      CommandSelector::MIDDLE_OF_DRIVER_STATION);
    chooser.AddOption("Three Note Source", CommandSelector::THREE_NOTE_SOURCE);
    chooser.AddOption("Middle Sub", CommandSelector::MIDDLE_SUB);
    chooser.AddOption("Three Middle Sub", CommandSelector::THREE_MIDDLE_SUB);
    chooser.AddOption("Amp Side Single", CommandSelector::AMP_SIDE_SINGLE);
    chooser.AddOption("Six Note", CommandSelector::SIX_NOTE);
    chooser.AddOption("Podium Note", CommandSelector::PODIUM_NOTE);
    chooser.AddOption("Ruin Ur Day", CommandSelector::RUIN_UR_DAY);

    frc::SmartDashboard::PutData("Auto Chooser", &chooser);
  }

  DrivebaseSubsystem& m_driveSub;
  ShooterSubsystem& m_shooterSub;
  IntakeSubsystem& m_intakeSub;
  DunkerSubsystem& m_dunkerSub;
  Vision& m_vision;

  frc::SendableChooser<CommandSelector> chooser;

  frc2::CommandPtr GetSelectedAutoCmd{frc2::cmd::None()};
};
}  // namespace autos
