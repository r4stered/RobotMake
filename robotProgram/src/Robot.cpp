// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#define LIBSSH_STATIC

#include <libssh/libssh.h>
#include <opencv2/core.hpp>
#include <fmt/format.h>
#include "frc/apriltag/AprilTag.h"
#include "frc/apriltag/AprilTagFieldLayout.h"
#include "frc/geometry/Pose3d.h"

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

using namespace frc;

int main() {
  ssh_session my_ssh_session = ssh_new();
  if (my_ssh_session == NULL)
    return -1;
  ssh_free(my_ssh_session);
  cv::Mat mat;
  fmt::print("Channels: {}\n", mat.channels());

  auto layout = AprilTagFieldLayout{
      std::vector<AprilTag>{
          AprilTag{1,
                   Pose3d{0_ft, 0_ft, 0_ft, Rotation3d{0_deg, 0_deg, 0_deg}}},
          AprilTag{
              2, Pose3d{4_ft, 4_ft, 4_ft, Rotation3d{0_deg, 0_deg, 180_deg}}}},
      54_ft, 27_ft};

  layout.SetOrigin(
      AprilTagFieldLayout::OriginPosition::kRedAllianceWallRightSide);

  auto mirrorPose =
      Pose3d{54_ft, 27_ft, 0_ft, Rotation3d{0_deg, 0_deg, 180_deg}};
  mirrorPose = Pose3d{50_ft, 23_ft, 4_ft, Rotation3d{0_deg, 0_deg, 0_deg}};

  auto inst = nt::NetworkTableInstance::Create();
  auto nt = inst.GetTable("containskey");
  nt->PutNumber("testkey", 5);
  nt::ResetInstance(inst.GetHandle());
  nt::NetworkTableInstance::Destroy(inst);
  return 0;
}