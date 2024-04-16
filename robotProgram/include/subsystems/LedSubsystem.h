// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <functional>

#include "str/LedStrip.h"

class LedSubsystem : public frc2::SubsystemBase {
 public:
  LedSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  frc2::CommandPtr SetSectionToColor(std::function<int()> section,
                                     std::function<double()> r,
                                     std::function<double()> g,
                                     std::function<double()> b);
  frc2::CommandPtr SetSectionToRainbow(std::function<int()> section);
  frc2::CommandPtr SetSectionToFade(std::function<int()> section,
                                    std::function<double()> r,
                                    std::function<double()> g,
                                    std::function<double()> b);
  frc2::CommandPtr SetSectionToTachometer(std::function<int()> section,
                                          std::function<double()> speed,
                                          std::function<double()> maxSpeed,
                                          frc::Color8Bit bottomColor,
                                          frc::Color8Bit topColor,
                                          bool reverse);
  frc2::CommandPtr SetSectionToKnightRider(std::function<int()> section,
                                           std::function<double()> r,
                                           std::function<double()> g,
                                           std::function<double()> b);
  frc2::CommandPtr SetSectionToChase(std::function<int()> section,
                                     std::function<double()> r,
                                     std::function<double()> g,
                                     std::function<double()> b);
  frc2::CommandPtr SetSectionToBlink(std::function<int()> section,
                                     std::function<double()> r,
                                     std::function<double()> g,
                                     std::function<double()> b,
                                     std::function<units::second_t()> onTime,
                                     std::function<units::second_t()> offTime);
  frc2::CommandPtr SetBothToBlinkYellow();
  frc2::CommandPtr SetBothToBlinkRed();
  frc2::CommandPtr SetBothToSolidGreen();
  frc2::CommandPtr SetBothToSolidOrange();
  frc2::CommandPtr SetBothToBlinkOrange();
  frc2::CommandPtr SetBothToOff();
  frc2::CommandPtr SetBothToRainbow();
  frc2::CommandPtr SetBothToTach(std::function<double()> currentSpeed,
                                 std::function<double()> setpoint);

 private:
  LedStrip ledStrip{};
};
