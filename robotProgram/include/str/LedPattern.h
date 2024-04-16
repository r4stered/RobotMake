// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/AddressableLED.h>

#include <iostream>
#include <vector>

class LedPattern {
 public:
  explicit LedPattern(int sectionLength) : size(sectionLength) {
    buffer.resize(size);
  }
  ~LedPattern() {}
  const std::vector<frc::AddressableLED::LEDData>& GetCurrentPattern() {
    return buffer;
  }
  virtual void Periodic() {}

 protected:
  int size = 0;
  std::vector<frc::AddressableLED::LEDData> buffer;
};
