// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <units/time.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace alert {
enum class AlertType { INFO, WARNING, CRITICAL };

class SendableAlert;

class Alert {
public:
  Alert(std::string group, std::string text, alert::AlertType type);
  Alert(std::string text, alert::AlertType type);
  void Set(bool active);
  void SetText(std::string text);
  alert::AlertType GetType() const { return type; }
  bool GetActive() const { return isActive; }
  units::second_t GetStartTime() const { return startActiveTime; }
  std::string GetText() const { return alertText; }

private:
  static std::unordered_map<std::string, alert::SendableAlert> groups;
  std::string alertText{};
  alert::AlertType type;
  bool isActive{false};
  units::second_t startActiveTime{0_s};
};

class SendableAlert : public wpi::Sendable {
public:
  SendableAlert() { }
  std::vector<std::string> GetStrings(alert::AlertType type);
  void InitSendable(wpi::SendableBuilder& builder) override;
  static std::vector<alert::Alert> alerts;
};
} // namespace alert
