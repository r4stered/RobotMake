// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#pragma once

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <limits>
#include <memory>
#include <utility>

#include "Constants.h"

class Vision {
 public:
  Vision() {
    frc::AprilTagFieldLayout layout =
        frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

    flPhotonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera(constants::vision::kflCameraName)),
        constants::vision::kflRobotToCam);
    flCamera = flPhotonEstimator->GetCamera();
    flCamera->SetVersionCheckEnabled(false);
    flPhotonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    frPhotonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera(constants::vision::kfrCameraName)),
        constants::vision::kfrRobotToCam);
    frCamera = frPhotonEstimator->GetCamera();
    frCamera->SetVersionCheckEnabled(false);
    frPhotonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    blPhotonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera(constants::vision::kblCameraName)),
        constants::vision::kblRobotToCam);
    blCamera = blPhotonEstimator->GetCamera();
    blCamera->SetVersionCheckEnabled(false);
    blPhotonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    brPhotonEstimator = std::make_unique<photon::PhotonPoseEstimator>(
        layout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
        std::move(photon::PhotonCamera(constants::vision::kbrCameraName)),
        constants::vision::kbrRobotToCam);
    brCamera = brPhotonEstimator->GetCamera();
    brCamera->SetVersionCheckEnabled(false);
    brPhotonEstimator->SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);

    noteCamera = std::make_shared<photon::PhotonCamera>(
        constants::vision::knoteCameraName);
    noteCamera->SetVersionCheckEnabled(false);

    if (frc::RobotBase::IsSimulation()) {
      visionSim = std::make_unique<photon::VisionSystemSim>("main");

      visionSim->AddAprilTags(layout);

      cameraProp = std::make_unique<photon::SimCameraProperties>();

      cameraProp->SetCalibration(1600, 1200, frc::Rotation2d{90_deg});
      cameraProp->SetCalibError(.35, .10);
      cameraProp->SetFPS(45_Hz);
      cameraProp->SetAvgLatency(20_ms);
      cameraProp->SetLatencyStdDev(15_ms);

      cameraSim = std::make_shared<photon::PhotonCameraSim>(flCamera.get(),
                                                            *cameraProp.get());

      visionSim->AddCamera(cameraSim.get(), constants::vision::kflRobotToCam);
      cameraSim->EnableDrawWireframe(true);
    }
  }

  photon::PhotonPipelineResult GetNoteCamResult() {
    return noteCamera->GetLatestResult();
  }

  photon::PhotonPipelineResult GetLatestResultFL() {
    return flCamera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetFLEstimatedGlobalPose() {
    auto visionEst = flPhotonEstimator->Update();
    units::second_t latestTimestamp =
        flCamera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - fllastEstTimestamp) > 1e-5_s;
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst.has_value()) {
        GetSimDebugField()
            .GetObject("FLVisionEstimation")
            ->SetPose(visionEst.value().estimatedPose.ToPose2d());
      } else {
        if (newResult) {
          GetSimDebugField().GetObject("FLVisionEstimation")->SetPoses({});
        }
      }
    }
    if (newResult) {
      fllastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  photon::PhotonPipelineResult GetLatestResultFR() {
    return frCamera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetFREstimatedGlobalPose() {
    auto visionEst = frPhotonEstimator->Update();
    units::second_t latestTimestamp =
        frCamera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - frlastEstTimestamp) > 1e-5_s;
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst.has_value()) {
        GetSimDebugField()
            .GetObject("FRVisionEstimation")
            ->SetPose(visionEst.value().estimatedPose.ToPose2d());
      } else {
        if (newResult) {
          GetSimDebugField().GetObject("FRVisionEstimation")->SetPoses({});
        }
      }
    }
    if (newResult) {
      frlastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  photon::PhotonPipelineResult GetLatestResultBL() {
    return blCamera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetBLEstimatedGlobalPose() {
    auto visionEst = blPhotonEstimator->Update();
    units::second_t latestTimestamp =
        blCamera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - bllastEstTimestamp) > 1e-5_s;
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst.has_value()) {
        GetSimDebugField()
            .GetObject("BLVisionEstimation")
            ->SetPose(visionEst.value().estimatedPose.ToPose2d());
      } else {
        if (newResult) {
          GetSimDebugField().GetObject("BLVisionEstimation")->SetPoses({});
        }
      }
    }
    if (newResult) {
      bllastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  photon::PhotonPipelineResult GetLatestResultBR() {
    return brCamera->GetLatestResult();
  }

  std::optional<photon::EstimatedRobotPose> GetBREstimatedGlobalPose() {
    auto visionEst = brPhotonEstimator->Update();
    units::second_t latestTimestamp =
        brCamera->GetLatestResult().GetTimestamp();
    bool newResult =
        units::math::abs(latestTimestamp - brlastEstTimestamp) > 1e-5_s;
    if (frc::RobotBase::IsSimulation()) {
      if (visionEst.has_value()) {
        GetSimDebugField()
            .GetObject("BRVisionEstimation")
            ->SetPose(visionEst.value().estimatedPose.ToPose2d());
      } else {
        if (newResult) {
          GetSimDebugField().GetObject("BRVisionEstimation")->SetPoses({});
        }
      }
    }
    if (newResult) {
      brlastEstTimestamp = latestTimestamp;
    }
    return visionEst;
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs =
        constants::vision::kSingleTagStdDevs;
    auto targetsfl = GetLatestResultFL().GetTargets();
    auto targetsfr = GetLatestResultFR().GetTargets();
    auto targetsbl = GetLatestResultBL().GetTargets();
    auto targetsbr = GetLatestResultBR().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targetsfl) {
      auto tagPose =
          flPhotonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    for (const auto& tgt : targetsfr) {
      auto tagPose =
          frPhotonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    for (const auto& tgt : targetsbl) {
      auto tagPose =
          blPhotonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    for (const auto& tgt : targetsbr) {
      auto tagPose =
          brPhotonEstimator->GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose.has_value()) {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = constants::vision::kMultiTagStdDevs;
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

  void SimPeriodic(frc::Pose2d robotSimPose) {
    visionSim->Update(robotSimPose);
  }

  void ResetSimPose(frc::Pose2d pose) {
    if (frc::RobotBase::IsSimulation()) {
      visionSim->ResetRobotPose(pose);
    }
  }

  int GetSpeakerCenterTagId() {
    auto ally = frc::DriverStation::GetAlliance();
    if (ally.has_value()) {
      if (ally.value() == frc::DriverStation::Alliance::kRed) {
        return 4;
      }
      if (ally.value() == frc::DriverStation::Alliance::kBlue) {
        return 7;
      }
    }
    return 7;
  }

  units::radian_t GetYawToCenterTag() {
    int tagToLookFor = GetSpeakerCenterTagId();
    photon::PhotonPipelineResult flResult = flCamera->GetLatestResult();
    photon::PhotonPipelineResult frResult = frCamera->GetLatestResult();
    if (flResult.HasTargets()) {
      for (const photon::PhotonTrackedTarget& target : flResult.GetTargets()) {
        if (target.GetFiducialId() == tagToLookFor) {
          return units::degree_t{target.GetYaw()} -
                 constants::vision::kflRobotToCam.Rotation().Z();
        }
      }
    }
    if (frResult.HasTargets()) {
      for (const photon::PhotonTrackedTarget& target : frResult.GetTargets()) {
        if (target.GetFiducialId() == tagToLookFor) {
          return units::degree_t{target.GetYaw()} -
                 constants::vision::kfrRobotToCam.Rotation().Z();
        }
      }
    }
    fmt::print("NO SUITABLE TARGET FOUND!!!!\n");
    return 0_rad;
  }

  frc::Field2d& GetSimDebugField() { return visionSim->GetDebugField(); }

 private:
  std::unique_ptr<photon::PhotonPoseEstimator> flPhotonEstimator;
  std::unique_ptr<photon::PhotonPoseEstimator> frPhotonEstimator;
  std::unique_ptr<photon::PhotonPoseEstimator> blPhotonEstimator;
  std::unique_ptr<photon::PhotonPoseEstimator> brPhotonEstimator;
  std::shared_ptr<photon::PhotonCamera> flCamera;
  std::shared_ptr<photon::PhotonCamera> frCamera;
  std::shared_ptr<photon::PhotonCamera> blCamera;
  std::shared_ptr<photon::PhotonCamera> brCamera;
  std::shared_ptr<photon::PhotonCamera> noteCamera;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;
  units::second_t fllastEstTimestamp{0_s};
  units::second_t frlastEstTimestamp{0_s};
  units::second_t bllastEstTimestamp{0_s};
  units::second_t brlastEstTimestamp{0_s};
};
