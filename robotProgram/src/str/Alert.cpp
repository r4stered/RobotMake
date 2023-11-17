// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/Alert.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <ranges>

using namespace alert;

std::unordered_map<std::string, alert::SendableAlert> alert::Alert::groups{};
std::vector<alert::Alert> alert::SendableAlert::alerts{};

Alert::Alert(std::string group, std::string text, AlertType type)
  : type(type)
{
  SetText(text);
  if (!groups.contains(group)) {
    groups[group] = SendableAlert();
    frc::SmartDashboard::PutData(group, &groups[group]);
  }
  groups[group].alerts.push_back(*this);
}

Alert::Alert(std::string text, AlertType type)
  : Alert("Alert", text, type)
{
}

void Alert::Set(bool active)
{
  if (active && !isActive) {
    startActiveTime = frc::Timer::GetFPGATimestamp();
    switch (type) {
    case AlertType::CRITICAL:
      frc::DataLogManager::Log(fmt::format("[CRITICAL]: {}", alertText));
      break;
    case AlertType::WARNING:
      frc::DataLogManager::Log(fmt::format("[WARNING]: {}", alertText));
      break;
    case AlertType::INFO:
      frc::DataLogManager::Log(fmt::format("[INFO]: {}", alertText));
      break;
    }
  }
  isActive = active;
}

void Alert::SetText(std::string text)
{
  if (isActive && !(text == alertText)) {
    startActiveTime = frc::Timer::GetFPGATimestamp();
    switch (type) {
    case AlertType::CRITICAL:
      frc::DataLogManager::Log(fmt::format("[CRITICAL]: {}", text));
      break;
    case AlertType::WARNING:
      frc::DataLogManager::Log(fmt::format("[WARNING]: {}", text));
      break;
    case AlertType::INFO:
      frc::DataLogManager::Log(fmt::format("[INFO]: {}", text));
      break;
    }
  }
  alertText = text;
}

std::vector<std::string> SendableAlert::GetStrings(AlertType type)
{
  // Take all the alerts and filter out based on input type and if they are
  // active. Then, sort based on start time. Finally transform the array of
  // alert into an array of strings containing the alert text
  std::vector<alert::Alert> validAlerts{};
  for (const auto& alert : alerts) {
    if (alert.GetType() == type && alert.GetActive()) {
      validAlerts.emplace_back(alert);
    }
  }
  std::sort(validAlerts.begin(), validAlerts.end(),
    [](const alert::Alert& a, const alert::Alert& b) {
      return a.GetStartTime() < b.GetStartTime();
    });

  std::vector<std::string> retVal{};
  for (const auto& alert : validAlerts) {
    retVal.emplace_back(alert.GetText());
  }

  return retVal;
}

void SendableAlert::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("Alerts");
  builder.AddStringArrayProperty(
    "errors", [this]() { return GetStrings(AlertType::CRITICAL); }, nullptr);
  builder.AddStringArrayProperty(
    "warnings", [this]() { return GetStrings(AlertType::WARNING); }, nullptr);
  builder.AddStringArrayProperty(
    "infos", [this]() { return GetStrings(AlertType::INFO); }, nullptr);
}
