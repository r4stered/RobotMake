// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/VisionSystemSim.h>
#include <photonlib/VisionTargetSim.h>

#include <memory>

class Vision {
public:
  Vision()
  {
    // photonEstimator.SetMultiTagFallbackStrategy(
    // photonlib::PoseStrategy::LOWEST_AMBIGUITY);

    cameraProp.SetCalibration(960, 720, frc::Rotation2d{90_deg});
    cameraProp.SetCalibError(.35, .10);
    cameraProp.SetFPS(15_Hz);
    cameraProp.SetAvgLatency(50_ms);
    cameraProp.SetLatencyStdDev(15_ms);

    cameraSim
      = std::make_shared<photonlib::PhotonCameraSim>(&camera, cameraProp);

    visionSim.AddCamera(cameraSim.get(), robotToCam);

    photonlib::VisionTargetSim testTarget{
      frc::Pose3d{frc::Translation3d{15.51_m, 1.07_m, 0.46_m},
        frc::Rotation3d{frc::Quaternion{0, 0, 0, 1}}},
      photonlib::kAprilTag16h5, 1};
    visionSim.AddVisionTargets({testTarget});

    cameraSim->EnableDrawWireframe(true);
  };

  void SimPeriodic(frc::Pose2d robotSimPose) { visionSim.Update(robotSimPose); }

private:
  frc::Transform3d robotToCam{
    frc::Translation3d{0.5_m, 0.5_m, 0.5_m}, frc::Rotation3d{}};
  photonlib::PhotonCamera camera{"photonvision"};
  // photonlib::PhotonPoseEstimator photonEstimator{
  //   LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp),
  //   photonlib::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, camera,
  //   robotToCam};
  photonlib::VisionSystemSim visionSim{"main"};
  photonlib::SimCameraProperties cameraProp{};
  std::shared_ptr<photonlib::PhotonCameraSim> cameraSim;
};
