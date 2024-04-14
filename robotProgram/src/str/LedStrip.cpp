// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/LedStrip.h"

#include "str/ledpatterns/SolidColorPattern.h"

LedStrip::LedStrip() {
  ledBuffer.fill(frc::AddressableLED::LEDData(255, 255, 255));
  leds.SetLength(34);
  leds.SetData(ledBuffer);
  leds.Start();
}

void LedStrip::AddSection(int subsectionLength) {
  if (sections.size() == 0) {
    sections.push_back(LedSection(0, subsectionLength));
  } else {
    int startIdx = sections[sections.size() - 1].GetLastLedIndex();
    sections.push_back(LedSection(startIdx, startIdx + subsectionLength));
  }
}

void LedStrip::FillBufferFromSections() {
  for (size_t i = 0; i < sections.size(); i++) {
    std::copy(sections[i].GetCurrentBuffer().begin(),
              sections[i].GetCurrentBuffer().end(),
              ledBuffer.begin() + sections[i].GetStartLedIndex());
  }
}

LedSection& LedStrip::GetSection(int idx) {
  return sections[idx];
}

void LedStrip::Periodic() {
  for (size_t i = 0; i < sections.size(); i++) {
    sections[i].Periodic();
  }
  FillBufferFromSections();
  leds.SetData(ledBuffer);
}
