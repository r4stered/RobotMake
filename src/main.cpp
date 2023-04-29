#include <iostream>
#include <frc/apriltag/AprilTagDetector.h>

int main(int argc, char* argv[]) {
  frc::AprilTagDetector detector;
  detector.AddFamily("tag16h5");
  detector.SetConfig({.refineEdges = false});
}