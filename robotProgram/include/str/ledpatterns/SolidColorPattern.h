// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/AddressableLED.h>

#include <vector>

#include "str/LedPattern.h"

class SolidColorPattern : public LedPattern {
 public:
  SolidColorPattern(frc::Color color, int sectionLength)
      : LedPattern(sectionLength) {
    std::fill(LedPattern::buffer.begin(), LedPattern::buffer.end(),
              frc::AddressableLED::LEDData(color.red * 255, color.green * 255,
                                           color.blue * 255));
  }
  ~SolidColorPattern() {}
  const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
    return LedPattern::buffer;
  }
  void Periodic() {}

 private:
  frc::Color currentColor;
};
