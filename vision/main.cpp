#include <cstdio>
#include <fmt/format.h>
#include "cscore.h"

int main(int charc, char *argv[])
{
    fmt::print("hostname: {}\n", cs::GetHostname());
    std::puts("IPv4 network addresses:");
    for (const auto &addr : cs::GetNetworkInterfaces())
    {
        fmt::print("  {}\n", addr);
    }
    cs::UsbCamera camera{"usbcam", 0};
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 1600, 1200, 50);
    cs::MjpegServer mjpegServer{"httpserver", 8081};
    mjpegServer.SetSource(camera);

    CS_Status status = 0;
    cs::AddListener(
        [&](const cs::RawEvent &event)
        {
            fmt::print("FPS={} MBPS={}\n", camera.GetActualFPS(),
                       (camera.GetActualDataRate() / 1000000.0));
        },
        cs::RawEvent::kTelemetryUpdated, false, &status);
    cs::SetTelemetryPeriod(1.0);

    std::getchar();
}