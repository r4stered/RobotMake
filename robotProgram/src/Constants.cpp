// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#include "Constants.h"

namespace constants {

namespace dunker {
const frc::TrapezoidProfile<units::radians>::Constraints
    PIVOT_CONTROLLER_CONSTRAINTS{constants::dunker::MAX_ROTATION_SPEED,
                                 constants::dunker::MAX_ROTATION_ACCEL};
}  // namespace dunker

namespace swerve {
namespace pathplanning {
const frc::TrapezoidProfile<units::radians>::Constraints
    GLOBAL_THETA_CONTROLLER_CONSTRAINTS{
        constants::swerve::physical::MAX_ROTATION_SPEED,
        constants::swerve::physical::MAX_ROTATION_ACCEL};
}  // namespace pathplanning
}  // namespace swerve
}  // namespace constants
