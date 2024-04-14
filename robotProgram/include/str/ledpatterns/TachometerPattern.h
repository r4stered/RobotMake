// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include "frc/AddressableLED.h"
#include "str/LedPattern.h"

class TachometerPattern : public LedPattern {
 public:
  TachometerPattern(std::function<double()> speed, double maxSpeed,
                    int sectionLength, frc::Color8Bit bottomColor,
                    frc::Color8Bit topColor, bool reverse)
      : LedPattern(sectionLength),
        currentSpeed(speed),
        maxSpeed(maxSpeed),
        bottomColor(bottomColor),
        topColor(topColor),
        reverse(reverse) {
    slopeRedLine =
        (topColor.red - bottomColor.red) / static_cast<double>(sectionLength);
    slopeGreenLine = (topColor.green - bottomColor.green) /
                     static_cast<double>(sectionLength);
    slopeBlueLine =
        (topColor.blue - bottomColor.blue) / static_cast<double>(sectionLength);

    std::fill(buffer.begin(), buffer.end(),
              frc::AddressableLED::LEDData(0.0, 0, 0.0));
  }
  ~TachometerPattern() {}
  const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
    return LedPattern::buffer;
  }
  void Periodic() override {
    double percentSpeed = currentSpeed() / maxSpeed;
    int lastIdx = std::clamp(percentSpeed, 0.0, 1.0) * (buffer.size() - 1);

    for (size_t i = 0; i < lastIdx; i++) {
      int idx = reverse ? buffer.size() - i - 1 : i;
      buffer[idx].SetRGB(slopeRedLine * i + bottomColor.red,
                         slopeGreenLine * i + bottomColor.green,
                         slopeBlueLine * i + bottomColor.blue);
    }
  }

 private:
  std::function<double()> currentSpeed = 0;
  frc::Color8Bit bottomColor{255, 0, 0};
  frc::Color8Bit topColor{0, 255, 0};
  double slopeRedLine = 0;
  double slopeGreenLine = 0;
  double slopeBlueLine = 0;
  double maxSpeed = 0;
  bool reverse{false};
};
