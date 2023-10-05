#include <opencv2/videoio.hpp>
#include "aruco_nano.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>
#include <fmt/format.h>
#include "aruco/markerdetector.h"
#include "aruco/calibrator.h"

static constexpr double MARKER_SIZE_METERS = 0.206375;

struct camera_calibration_data
{
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeff;
};

camera_calibration_data read_calibration_data(std::string_view filePath)
{
    // TODO: Actually read from file
    return camera_calibration_data{};
}

void calibrate_camera(std::string devicePath, nt::BooleanEntry &takePicSignal, nt::BooleanEntry &stopSignal, nt::StringEntry &infoSignal)
{
    cv::VideoCapture vidCap;
    if (!vidCap.open(devicePath, cv::CAP_DSHOW))
    {
        fmt::print("Video Capture couldn't open!\n");
    }
    cv::Mat inputImage;
    vidCap >> inputImage;
    aruco::Calibrator calibrator;
    calibrator.setParams(inputImage.size(), MARKER_SIZE_METERS, "aruco_calibration_grid_board_a4.yml");
    aruco::MarkerDetector detector;
    do
    {
        vidCap.retrieve(inputImage);
        std::vector<aruco::Marker> detectedMarkers = detector.detect(inputImage);
        for (const auto &m : detectedMarkers)
        {
            m.draw(inputImage);
        }

        if (takePicSignal.Get())
        {
            takePicSignal.Set(false);
            calibrator.addView(detectedMarkers);
        }
    } while (!stopSignal.Get());
    stopSignal.Set(false);
    aruco::CameraParameters cameraParams;
    if (calibrator.getCalibrationResults(cameraParams))
    {
        cameraParams.saveToFile("camera_calibration_results.yml");
        infoSignal.Set("Reprojection Error: " + std::to_string(calibrator.getReprjError()) + "\n");
    }
}

int main(int charc, char *argv[])
{
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("cyphercam");
    nt::BooleanEntry startCalibrationEntry = table->GetBooleanTopic("start_calibration").GetEntry(false);
    nt::BooleanEntry stopCalibrationEntry = table->GetBooleanTopic("stop_calibration").GetEntry(false);
    nt::BooleanEntry takePicEntry = table->GetBooleanTopic("take_picture").GetEntry(false);
    nt::StringEntry infoEntry = table->GetStringTopic("info_msg").GetEntry("");

    camera_calibration_data calibration_data_cam_one = read_calibration_data("cam_one_cal_data.yml");
    camera_calibration_data calibration_data_cam_two = read_calibration_data("cam_two_cal_data.yml");

    cv::VideoCapture vidCap;
    if (!vidCap.open(0))
    {
        infoEntry.Set("Video Capture couldn't open!\n");
    }
    else
    {
        fmt::print("video capture opened!\n");
    }

    bool capturingFrames = true;

    std::vector<aruconano::Marker> markersFromCurrentFrame;
    cv::Mat capturedFrame;
    while (capturingFrames)
    {
        startCalibrationEntry.Set(true);
        if (startCalibrationEntry.Get())
        {
            vidCap.release();
            startCalibrationEntry.Set(false);
            calibrate_camera("", takePicEntry, stopCalibrationEntry, infoEntry);
        }

        if (!vidCap.read(capturedFrame))
        {
            infoEntry.Set("Couldn't grab camera frame!");
        }

        markersFromCurrentFrame = aruconano::MarkerDetector::detect(capturedFrame, 10U, aruconano::MarkerDetector::APRILTAG_36h11);
        for (const aruconano::Marker &marker : markersFromCurrentFrame)
        {
            // Is the marker size supposed to be cm? or just overall frame percentage???
            auto pose = marker.estimatePose(calibration_data_cam_one.cameraMatrix, calibration_data_cam_one.distortionCoeff, MARKER_SIZE_METERS);
        }

        cv::imshow("test", capturedFrame);
        cv::waitKey(1);
    }
    return 0;
}