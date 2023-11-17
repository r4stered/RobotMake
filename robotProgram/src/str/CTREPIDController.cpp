// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "str/CTREPIDController.h"

#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include <algorithm>
#include <cmath>

#include "frc/MathUtil.h"
#include "wpimath/MathShared.h"

using namespace frc;

CTREPIDController::CTREPIDController(double Kp, double Ki, double Kd)
  : m_Kp(Kp)
  , m_Ki(Ki)
  , m_Kd(Kd)
{
  static int instances = 0;
  instances++;

  wpi::math::MathSharedStore::ReportUsage(
    wpi::math::MathUsageId::kController_PIDController2, instances);
  wpi::SendableRegistry::Add(this, "CTREPIDController", instances);
}

void CTREPIDController::SetPID(double Kp, double Ki, double Kd)
{
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;
}

void CTREPIDController::SetP(double Kp) { m_Kp = Kp; }

void CTREPIDController::SetI(double Ki) { m_Ki = Ki; }

void CTREPIDController::SetD(double Kd) { m_Kd = Kd; }

double CTREPIDController::GetP() const { return m_Kp; }

double CTREPIDController::GetI() const { return m_Ki; }

double CTREPIDController::GetD() const { return m_Kd; }

double CTREPIDController::GetPositionTolerance() const
{
  return m_positionTolerance;
}

double CTREPIDController::GetVelocityTolerance() const
{
  return m_velocityTolerance;
}

double CTREPIDController::GetSetpoint() const { return m_setpoint; }

bool CTREPIDController::AtSetpoint() const
{
  return m_haveMeasurement && m_haveSetpoint
    && std::abs(m_positionError) < m_positionTolerance
    && std::abs(m_velocityError) < m_velocityTolerance;
}

void CTREPIDController::EnableContinuousInput(
  double minimumInput, double maximumInput)
{
  m_continuous = true;
  m_minimumInput = minimumInput;
  m_maximumInput = maximumInput;
}

void CTREPIDController::DisableContinuousInput() { m_continuous = false; }

bool CTREPIDController::IsContinuousInputEnabled() const
{
  return m_continuous;
}

void CTREPIDController::SetIntegratorRange(
  double minimumIntegral, double maximumIntegral)
{
  m_minimumIntegral = minimumIntegral;
  m_maximumIntegral = maximumIntegral;
}

void CTREPIDController::SetTolerance(
  double positionTolerance, double velocityTolerance)
{
  m_positionTolerance = positionTolerance;
  m_velocityTolerance = velocityTolerance;
}

double CTREPIDController::GetPositionError() const { return m_positionError; }

double CTREPIDController::GetVelocityError() const { return m_velocityError; }

double CTREPIDController::Calculate(
  double measurement, double setpoint, units::second_t currentTimestamp)
{
  m_setpoint = setpoint;
  m_haveSetpoint = true;
  m_measurement = measurement;
  m_prevError = m_positionError;
  m_haveMeasurement = true;

  units::second_t thisPeriod = currentTimestamp - m_lastTimestamp;
  m_lastTimestamp = currentTimestamp;

  if (m_continuous) {
    double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
    m_positionError
      = InputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
  } else {
    m_positionError = m_setpoint - m_measurement;
  }

  m_velocityError = (m_positionError - m_prevError) / thisPeriod.value();

  if (m_Ki != 0) {
    m_totalError
      = std::clamp(m_totalError + m_positionError * thisPeriod.value(),
        m_minimumIntegral / m_Ki, m_maximumIntegral / m_Ki);
  }

  m_lastAppliedOutput
    = m_Kp * m_positionError + m_Ki * m_totalError + m_Kd * m_velocityError;
  return m_lastAppliedOutput;
}

void CTREPIDController::Reset()
{
  m_positionError = 0;
  m_prevError = 0;
  m_totalError = 0;
  m_velocityError = 0;
  m_haveMeasurement = false;
}

double CTREPIDController::GetLastAppliedOutput() { return m_lastAppliedOutput; }

void CTREPIDController::InitSendable(wpi::SendableBuilder& builder)
{
  builder.SetSmartDashboardType("CTREPIDController");
  builder.AddDoubleProperty(
    "p", [this] { return GetP(); }, [this](double value) { SetP(value); });
  builder.AddDoubleProperty(
    "i", [this] { return GetI(); }, [this](double value) { SetI(value); });
  builder.AddDoubleProperty(
    "d", [this] { return GetD(); }, [this](double value) { SetD(value); });
  builder.AddDoubleProperty(
    "setpoint", [this] { return GetSetpoint(); }, nullptr);
}
