// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <str/Units.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>

namespace constants {

static constexpr units::second_t ROBOT_DT = 0.02_s;

namespace motor_info {
  static constexpr units::radians_per_second_t FALCON_FOC_FREE_SPEED = 6080_rpm;
} // namespace motor_info

namespace drivebase {

  namespace gains {
    static constexpr double DRIVE_KA = 0.0;
    static constexpr double DRIVE_KV = 0.0;
    static constexpr double DRIVE_KS = 0.0;
    static constexpr double DRIVE_KP = 3.0;
    static constexpr double DRIVE_KI = 0.0;
    static constexpr double DRIVE_KD = 0.0;

    static constexpr double STEER_KA = 0.0;
    static constexpr double STEER_KV = 0.0;
    static constexpr double STEER_KS = 0.0;
    static constexpr double STEER_KP = 1.0;
    static constexpr double STEER_KI = 0.0;
    static constexpr double STEER_KD = 0.00;

    static constexpr double STEER_MOTION_MAGIC_ACCEL = 100.0;
    static constexpr double STEER_MOTION_MAGIC_CRUISE_VEL = 10.0;

    static constexpr double TRANSLATION_P = 10.0;
    static constexpr double TRANSLATION_I = 0.0;
    static constexpr double TRANSLATION_D = 0.0;

    static constexpr double ROTATION_P = 10.0;
    static constexpr double ROTATION_I = 0.0;
    static constexpr double ROTATION_D = 0.0;
  } // namespace gains

  namespace can {
    static constexpr int FL_DRIVE = 2;
    static constexpr int FL_STEER = 3;
    static constexpr int FL_ENC = 4;

    static constexpr int FR_DRIVE = 5;
    static constexpr int FR_STEER = 6;
    static constexpr int FR_ENC = 7;

    static constexpr int BL_DRIVE = 8;
    static constexpr int BL_STEER = 9;
    static constexpr int BL_ENC = 10;

    static constexpr int BR_DRIVE = 11;
    static constexpr int BR_STEER = 12;
    static constexpr int BR_ENC = 13;

    static constexpr int IMU = 14;
  } // namespace can

  namespace physical {
    static constexpr auto WHEELBASE_LENGTH
      = 25_in; // From front wheel to back wheel
    static constexpr auto WHEELBASE_WIDTH
      = 25_in; // From left wheel to right wheel
    static constexpr auto WHEEL_DIAM = 4_in;
    static constexpr double DRIVE_GEARING
      = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // L2 SDS
    static constexpr double STEER_GEARING
      = (50.0 / 14.0) * (10.0 / 60.0); // SDS Steer Ratio
    static constexpr double DRIVE_STEER_COUPLING = (50.0 / 14.0);
    static constexpr double STEER_ENC_GEARING = (1.0);
    static constexpr units::ampere_t SLIP_CURRENT = 400_A;
    static constexpr units::meters_per_second_t MAX_DRIVE_SPEED{
      str::Units::ConvertAngularVelocityToLinearVelocity(
        motor_info::FALCON_FOC_FREE_SPEED / DRIVE_GEARING, WHEEL_DIAM / 2)};
    static constexpr units::degrees_per_second_t MAX_ROTATION_SPEED{
      360_deg_per_s};
    static constexpr double FL_ENC_OFFSET = 0;
    static constexpr double FR_ENC_OFFSET = 0;
    static constexpr double BL_ENC_OFFSET = 0;
    static constexpr double BR_ENC_OFFSET = 0;
  } // namespace physical
} // namespace drivebase
} // namespace constants
