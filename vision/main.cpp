#include <opencv2/videoio.hpp>
#include "aruco_nano.h"
#include <networktables/NetworkTableInstance.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>
#include <fmt/format.h>
#include "aruco/markerdetector.h"
#include "aruco/calibrator.h"
#include <cscore.h>
#include <cscore_cv.h>

static constexpr double MARKER_SIZE_METERS = 0.206375;

cv::VideoCapture OpenCamera()
{
    cv::VideoCapture vidCap;
    if (!vidCap.open(0))
    {
        fmt::print("Video Capture couldn't open!\n");
    }
    else
    {
        fmt::print("video capture opened!\n");
        vidCap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        vidCap.set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        vidCap.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
        vidCap.set(cv::CAP_PROP_FPS, 50);
    }
    return vidCap;
}

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

void calibrate_camera(std::string devicePath, nt::BooleanEntry &takePicSignal, nt::BooleanEntry &stopSignal, cs::CvSource &frameSource, nt::BooleanEntry &picRecv)
{
    cv::VideoCapture vidCap = OpenCamera();
    cv::Mat inputImage;
    vidCap >> inputImage;
    aruco::Calibrator calibrator;
    calibrator.setParams(inputImage.size(), 0.037, "aruco_calibration_grid_board_a4.yml");
    aruco::MarkerDetector detector;
    do
    {
        vidCap >> inputImage;
        std::vector<aruco::Marker> detectedMarkers = detector.detect(inputImage);
        for (const auto &m : detectedMarkers)
        {
            m.draw(inputImage);
        }

        if (takePicSignal.Get() && !picRecv.Get())
        {
            fmt::print("Picture taken!\n");
            picRecv.Set(true);
            calibrator.addView(detectedMarkers);
        }
        frameSource.PutFrame(inputImage);
    } while (!stopSignal.Get());
    stopSignal.Set(false);
    aruco::CameraParameters cameraParams;
    if (calibrator.getCalibrationResults(cameraParams))
    {
        cameraParams.saveToFile("camera_calibration_results.yml");
        fmt::print("Reprojection Error: {} \n", calibrator.getReprjError());
    }
    else
    {
        fmt::print("Camera Calibration Failed!\n");
    }
    vidCap.release();
}

int main(int charc, char *argv[])
{
    nt::NetworkTableInstance ntInstance = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = ntInstance.GetTable("cyphercam");
    nt::BooleanEntry startCalibrationEntry = table->GetBooleanTopic("start_calibration").GetEntry(false);
    nt::BooleanEntry stopCalibrationEntry = table->GetBooleanTopic("stop_calibration").GetEntry(false);
    nt::BooleanEntry takePicEntry = table->GetBooleanTopic("take_picture").GetEntry(false);
    nt::BooleanEntry pictureRecv = table->GetBooleanTopic("pic_recv").GetEntry(false);
    nt::StringEntry infoEntry = table->GetStringTopic("info_msg").GetEntry("");
    ntInstance.StartClient4("opi4");
    ntInstance.SetServer("192.168.1.93");

    cs::CvSource visionOutput("vision_output", cs::VideoMode::kMJPEG, 1600, 1200, 50);
    cs::MjpegServer camera_stream("opi_cam", 1181);
    camera_stream.SetSource(visionOutput);

    camera_calibration_data calibration_data_cam_one = read_calibration_data("cam_one_cal_data.yml");
    camera_calibration_data calibration_data_cam_two = read_calibration_data("cam_two_cal_data.yml");

    bool capturingFrames = true;

    std::vector<aruconano::Marker> markersFromCurrentFrame;
    cv::Mat capturedFrame;

    cv::VideoCapture vidCap = OpenCamera();
    while (capturingFrames)
    {
        int64 startTickCount = cv::getTickCount();
        if (startCalibrationEntry.Get())
        {
            vidCap.release();
            startCalibrationEntry.Set(false);
            calibrate_camera("", takePicEntry, stopCalibrationEntry, visionOutput, pictureRecv);
            vidCap = OpenCamera();
        }

        if (!vidCap.read(capturedFrame))
        {
            fmt::print("Couldn't grab camera frame!");
        }

        markersFromCurrentFrame = aruconano::MarkerDetector::detect(capturedFrame, 10U, aruconano::MarkerDetector::APRILTAG_36h11);
        for (const aruconano::Marker &marker : markersFromCurrentFrame)
        {
            // Is the marker size supposed to be cm? or just overall frame percentage???
            marker.draw(capturedFrame);
            auto pose = marker.estimatePose(calibration_data_cam_one.cameraMatrix, calibration_data_cam_one.distortionCoeff, MARKER_SIZE_METERS);
        }

        double fps = cv::getTickFrequency() / (cv::getTickCount() - startTickCount);
        cv::putText(capturedFrame, std::to_string(fps), cv::Point(0, capturedFrame.size().height - 1), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 0, 255));
        visionOutput.PutFrame(capturedFrame);
    }
    return 0;
}