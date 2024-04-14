// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "subsystems/LedSubsystem.h"

#include <frc2/command/Commands.h>

#include "str/ledpatterns/BlinkPattern.h"
#include "str/ledpatterns/ChasePattern.h"
#include "str/ledpatterns/FadePattern.h"
#include "str/ledpatterns/KnightRiderPattern.h"
#include "str/ledpatterns/RainbowPattern.h"
#include "str/ledpatterns/TachometerPattern.h"

LedSubsystem::LedSubsystem() {
  ledStrip.AddSection(17);
  ledStrip.AddSection(17);
}

// This method will be called once per scheduler run
void LedSubsystem::Periodic() {
  ledStrip.Periodic();
}

frc2::CommandPtr LedSubsystem::SetSectionToColor(std::function<int()> section,
                                                 std::function<double()> r,
                                                 std::function<double()> g,
                                                 std::function<double()> b) {
  return frc2::cmd::RunOnce(
             [this, section, r, g, b] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<SolidColorPattern>(SolidColorPattern(
                       frc::Color(r(), g(), b()),
                       ledStrip.GetSection(section()).GetLength()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToRainbow(
    std::function<int()> section) {
  return frc2::cmd::RunOnce(
             [this, section] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<RainbowPattern>(RainbowPattern(
                       ledStrip.GetSection(section()).GetLength()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToFade(std::function<int()> section,
                                                std::function<double()> r,
                                                std::function<double()> g,
                                                std::function<double()> b) {
  return frc2::cmd::RunOnce(
             [this, section, r, g, b] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<FadePattern>(
                       FadePattern(frc::Color(r(), g(), b()),
                                   ledStrip.GetSection(section()).GetLength()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToTachometer(
    std::function<int()> section, std::function<double()> speed,
    std::function<double()> maxSpeed, frc::Color8Bit bottomColor,
    frc::Color8Bit topColor, bool reverse) {
  return frc2::cmd::RunOnce(
             [this, section, speed, maxSpeed, bottomColor, topColor, reverse] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<TachometerPattern>(TachometerPattern(
                       speed, maxSpeed(),
                       ledStrip.GetSection(section()).GetLength(), bottomColor,
                       topColor, reverse));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToKnightRider(
    std::function<int()> section, std::function<double()> r,
    std::function<double()> g, std::function<double()> b) {
  return frc2::cmd::RunOnce(
             [this, section, r, g, b] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<KnightRiderPattern>(KnightRiderPattern(
                       frc::Color(r(), g(), b()),
                       ledStrip.GetSection(section()).GetLength()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToChase(std::function<int()> section,
                                                 std::function<double()> r,
                                                 std::function<double()> g,
                                                 std::function<double()> b) {
  return frc2::cmd::RunOnce(
             [this, section, r, g, b] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<ChasePattern>(ChasePattern(
                       frc::Color(r(), g(), b()),
                       ledStrip.GetSection(section()).GetLength()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetSectionToBlink(
    std::function<int()> section, std::function<double()> r,
    std::function<double()> g, std::function<double()> b,
    std::function<units::second_t()> onTime,
    std::function<units::second_t()> offTime) {
  return frc2::cmd::RunOnce(
             [this, section, r, g, b, onTime, offTime] {
               std::unique_ptr<LedPattern> pattern =
                   std::make_unique<BlinkPattern>(
                       BlinkPattern(frc::Color(r(), g(), b()),
                                    ledStrip.GetSection(section()).GetLength(),
                                    onTime(), offTime()));
               ledStrip.GetSection(section()).SetPattern(std::move(pattern));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToBlinkYellow() {
  return frc2::cmd::RunOnce(
             [this] {
               std::unique_ptr<LedPattern> pattern1 =
                   std::make_unique<BlinkPattern>(BlinkPattern(
                       frc::Color(1.0, 1.0, 0.0),
                       ledStrip.GetSection(0).GetLength(), .1_s, .1_s));
               std::unique_ptr<LedPattern> pattern2 =
                   std::make_unique<BlinkPattern>(BlinkPattern(
                       frc::Color(1.0, 1.0, 0.0),
                       ledStrip.GetSection(1).GetLength(), .1_s, .1_s));
               ledStrip.GetSection(0).SetPattern(std::move(pattern1));
               ledStrip.GetSection(1).SetPattern(std::move(pattern2));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToBlinkRed() {
  return frc2::cmd::RunOnce(
             [this] {
               std::unique_ptr<LedPattern> pattern1 =
                   std::make_unique<BlinkPattern>(BlinkPattern(
                       frc::Color(1.0, 0.0, 0.0),
                       ledStrip.GetSection(0).GetLength(), .1_s, .1_s));
               std::unique_ptr<LedPattern> pattern2 =
                   std::make_unique<BlinkPattern>(BlinkPattern(
                       frc::Color(1.0, 0.0, 0.0),
                       ledStrip.GetSection(1).GetLength(), .1_s, .1_s));
               ledStrip.GetSection(0).SetPattern(std::move(pattern1));
               ledStrip.GetSection(1).SetPattern(std::move(pattern2));
             },
             {this})
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToSolidGreen() {
  return frc2::cmd::Sequence(
             SetSectionToColor([] { return 0; }, [] { return 0.0; },
                               [] { return 1.0; }, [] { return 0.0; }),
             SetSectionToColor([] { return 1; }, [] { return 0.0; },
                               [] { return 1.0; }, [] { return 0.0; }))
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToSolidOrange() {
  return frc2::cmd::Sequence(
             SetSectionToColor([] { return 0; }, [] { return 0.98; },
                               [] { return .46; }, [] { return 0.011; }),
             SetSectionToColor([] { return 1; }, [] { return 0.98; },
                               [] { return .46; }, [] { return 0.011; }))
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToOff() {
  return frc2::cmd::Sequence(
             SetSectionToColor([] { return 0; }, [] { return 0.0; },
                               [] { return 0; }, [] { return 0.0; }),
             SetSectionToColor([] { return 1; }, [] { return 0.0; },
                               [] { return 0; }, [] { return 0.0; }))
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToBlinkOrange() {
  return frc2::cmd::Sequence(
             SetSectionToColor([] { return 0; }, [] { return 0.98; },
                               [] { return .46; }, [] { return 0.011; }),
             SetSectionToColor([] { return 1; }, [] { return 0.98; },
                               [] { return .46; }, [] { return 0.011; }))
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToRainbow() {
  return frc2::cmd::Sequence(SetSectionToRainbow([] { return 0; }),
                             SetSectionToRainbow([] { return 1; }))
      .IgnoringDisable(true);
}

frc2::CommandPtr LedSubsystem::SetBothToTach(
    std::function<double()> currentSpeed, std::function<double()> setpoint) {
  return frc2::cmd::Sequence(
             SetSectionToTachometer([] { return 0; }, currentSpeed, setpoint,
                                    frc::Color8Bit{255, 0, 0},
                                    frc::Color8Bit{0, 255, 0}, false),
             SetSectionToTachometer([] { return 1; }, currentSpeed, setpoint,
                                    frc::Color8Bit{255, 0, 0},
                                    frc::Color8Bit{0, 255, 0}, true))
      .IgnoringDisable(true);
}
