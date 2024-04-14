// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/MathUtil.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <numbers>

namespace units {

template <typename T>
bool essentiallyEqual(T a, T b, units::scalar_t epsilon) {
  return units::math::abs(a - b) <=
         ((units::math::abs(a) > units::math::abs(b) ? units::math::abs(b)
                                                     : units::math::abs(a)) *
          epsilon);
}

// SHOOTER
using radian_volt_kp_unit =
    units::compound_unit<units::volt, units::inverse<units::radians>>;
using radian_volt_kp_unit_t = units::unit_t<radian_volt_kp_unit>;

using radian_volt_ki_unit = units::compound_unit<
    units::volt,
    units::inverse<units::compound_unit<units::radian, units::seconds>>>;
using radian_volt_ki_unit_t = units::unit_t<radian_volt_ki_unit>;

using radian_volt_kd_unit =
    units::compound_unit<units::volt,
                         units::inverse<units::radians_per_second>>;
using radian_volt_kd_unit_t = units::unit_t<radian_volt_kd_unit>;

using meter_volt_kp_unit =
    units::compound_unit<units::volt, units::inverse<units::meters>>;
using meter_volt_kp_unit_t = units::unit_t<meter_volt_kp_unit>;

using meter_volt_ki_unit = units::compound_unit<
    units::volt,
    units::inverse<units::compound_unit<units::meter, units::seconds>>>;
using meter_volt_ki_unit_t = units::unit_t<meter_volt_ki_unit>;

using meter_volt_kd_unit =
    units::compound_unit<units::volt, units::inverse<units::meters_per_second>>;
using meter_volt_kd_unit_t = units::unit_t<meter_volt_kd_unit>;

constexpr units::meter_t ConvertEncoderTicksToDistance(
    int ticks, int encoderResolution, double gearing,
    units::meter_t wheelRadius) {
  return (ticks / (encoderResolution * gearing)) *
         (2 * std::numbers::pi * wheelRadius);
}

constexpr units::meter_t ConvertAngularDistanceToLinearDistance(
    units::radian_t turns, units::meter_t wheelRadius) {
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return (turns / 1_rad) * wheelRadius;
}

constexpr units::radian_t ConvertLinearDistanceToAngularDistance(
    units::meter_t distance, units::meter_t wheelRadius) {
  return (distance / wheelRadius) * 1_rad;
}

constexpr units::radian_t ConvertTicksToAngle(int ticks, int encoderResolution,
                                              double gearing,
                                              bool wrap = true) {
  auto angle = (ticks / (encoderResolution * gearing)) *
               units::radian_t{2 * std::numbers::pi};
  return wrap ? frc::AngleModulus(angle) : angle;
}

constexpr int ConvertDistanceToEncoderTicks(units::meter_t distance,
                                            int encoderResolution,
                                            double gearing,
                                            units::meter_t wheelRadius) {
  return distance * (encoderResolution * gearing) /
         (std::numbers::pi * 2 * wheelRadius);
}

constexpr int ConvertAngleToEncoderTicks(units::radian_t angle,
                                         int encoderResolution, double gearing,
                                         bool wrap = true) {
  if (wrap) {
    angle = frc::AngleModulus(angle);
  }
  return angle * (encoderResolution * gearing) /
         units::radian_t{std::numbers::pi * 2};
}

constexpr units::radians_per_second_t ConvertLinearVelocityToAngularVelocity(
    units::meters_per_second_t linearVelocity, units::meter_t radius) {
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return linearVelocity / (radius / 1_rad);
}

constexpr units::meters_per_second_t ConvertAngularVelocityToLinearVelocity(
    units::radians_per_second_t angularVelocity, units::meter_t radius) {
  // https://www.calculussolution.com/blog/arc-length
  // radians are actually dimensionless
  return angularVelocity * (radius / 1_rad);
}

template <typename T>
  requires std::is_arithmetic_v<T> || units::traits::is_unit_t_v<T>
constexpr T Map(T input, T minIn, T maxIn, T minOut, T maxOut) {
  return (input - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
}

template <typename T>
constexpr int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

}  // namespace units
