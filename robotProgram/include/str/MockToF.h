// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/length.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <TimeOfFlight.h>

class MockToF : public wpi::Sendable, public wpi::SendableHelper<MockToF> {
 public:
  explicit MockToF(int port);
  void IdentifySensor();
  int GetFirmwareVersion() const;
  int GetSerialNumber() const;
  bool IsRangeValid() const;
  double GetRange() const;
  double GetRangeSigma() const;
  double GetAmbientLightLevel() const;
  frc::TimeOfFlight::Status GetStatus() const;
  void SetRangingMode(frc::TimeOfFlight::RangingMode mode, int sensorPeriod);
  void SetRangeOfInterest(int topLeftX, int topLeftY, int bottomRightX,
                          int bottomRightY);
  void SetDistance(units::millimeter_t dist);
  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  int portNum;
  int tlX{0};
  int tlY{0};
  int brX{1};
  int brY{1};
  frc::TimeOfFlight::RangingMode currentMode{
      frc::TimeOfFlight::RangingMode::kShort};
  double currentDistance{0};
};
