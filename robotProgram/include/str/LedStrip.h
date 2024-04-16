// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/AddressableLED.h>

#include <array>
#include <vector>

#include "LedSection.h"

class LedStrip {
 public:
  LedStrip();
  void AddSection(int subsectionLength);
  void Periodic();
  LedSection& GetSection(int idx);

 private:
  void FillBufferFromSections();
  std::vector<LedSection> sections;
  frc::AddressableLED leds{0};
  std::array<frc::AddressableLED::LEDData, 34> ledBuffer;
};
