#pragma once

#include <frc/MathUtil.h>
#include <numbers>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace str
{
  class Units
  {
  public:
    static constexpr units::meter_t ConvertEncoderTicksToDistance(
        double ticks,
        int encoderResolution,
        double gearing,
        units::meter_t wheelRadius)
    {
      return (ticks / (encoderResolution * gearing)) * (2 * std::numbers::pi * wheelRadius);
    }

    static constexpr units::meter_t ConvertAngularDistanceToLinearDistance(
        units::radian_t turns,
        units::meter_t wheelRadius)
    {
      return units::meter_t{turns.to<double>() * wheelRadius.to<double>()};
    }

    static constexpr units::radian_t ConvertTicksToAngle(
        double ticks,
        int encoderResolution,
        double gearing,
        bool wrap = true)
    {
      units::radian_t retVal = units::radian_t((ticks / (encoderResolution * gearing)) * (2 * std::numbers::pi));
      return wrap ? frc::AngleModulus(retVal) : retVal;
    }

    static constexpr double ConvertDistanceToEncoderTicks(
        units::meter_t distance,
        int encoderResolution,
        double gearing,
        units::meter_t wheelRadius)
    {
      return distance * (encoderResolution * gearing) / (std::numbers::pi * 2 * wheelRadius);
    }

    static constexpr double ConvertAngleToEncoderTicks(
        units::radian_t angle,
        int encoderResolution,
        double gearing,
        bool wrap = true)
    {
      if (wrap)
      {
        angle = frc::AngleModulus(angle);
      }
      return angle.to<double>() * (encoderResolution * gearing) / (std::numbers::pi * 2);
    }

    static constexpr units::radians_per_second_t ConvertLinearVelocityToAngularVelocity(
        units::meters_per_second_t linearVelocity,
        units::meter_t radius)
    {
      return units::radians_per_second_t(linearVelocity.to<double>() / radius.to<double>());
    }

    static constexpr units::meters_per_second_t ConvertAngularVelocityToLinearVelocity(
        units::radians_per_second_t angularVelocity,
        units::meter_t radius)
    {
      return units::meters_per_second_t(angularVelocity.to<double>() * radius.to<double>());
    }

    // TODO: implement our own abs function as std::abs isnt constexpr until c++23
    static double Deadband(double input, double deadband)
    {
      if (std::abs(input) > deadband)
      {
        if (input > 0.0)
        {
          return (input - deadband) / (1.0 - deadband);
        }
        else
        {
          return (input + deadband) / (1.0 - deadband);
        }
      }
      else
      {
        return 0.0;
      }
    }

    static constexpr double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    template <typename T>
    static constexpr int sgn(T val)
    {
      return (T(0) < val) - (val < T(0));
    }

  private:
  };
}