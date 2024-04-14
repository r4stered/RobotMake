// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <vector>

#include "frc/AddressableLED.h"
#include "str/LedPattern.h"

class FadePattern : public LedPattern {
 public:
  FadePattern(frc::Color8Bit color, int sectionLength)
      : LedPattern(sectionLength), fadeColor(color) {
    std::fill(buffer.begin(), buffer.end(),
              frc::AddressableLED::LEDData(1.0, 0, 1.0));
  }
  ~FadePattern() {}
  const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
    return LedPattern::buffer;
  }
  void Periodic() override {
    r = (kup / 256.0) * fadeColor.red;
    g = (kup / 256.0) * fadeColor.green;
    b = (kup / 256.0) * fadeColor.blue;
    for (size_t i = 0; i < buffer.size(); i++) {
      buffer[i].SetRGB(r, g, b);
    }
    if (kup == 255) {
      hitUpper = true;
    }
    if (hitUpper) {
      kup -= 5;
      if (kup == 0) {
        hitUpper = false;
      }
    } else {
      kup += 5;
    }
  }

 private:
  bool hitUpper = false;
  double kup = 0;
  double r = 0;
  double g = 0;
  double b = 0;
  frc::Color8Bit fadeColor;
};
