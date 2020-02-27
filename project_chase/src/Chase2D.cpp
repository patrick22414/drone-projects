#include "Chase2D.h"

Chase2D::Chase2D(const std::string& connection, int camera, cv::VideoCaptureAPIs api, const std::string& video_output)
{
    // TODO

    this->capture = cv::VideoCapture(camera, api);

    mavsdk::Mavsdk dc;

    dc.add_any_connection(connection);

    mavsdk::System& system = dc.system();

    this->action    = std::make_shared<mavsdk::Action>(system);
    this->offboard  = std::make_shared<mavsdk::Offboard>(system);
    this->telemetry = std::make_shared<mavsdk::Telemetry>(system);

    Chase2D::log("Constructor complete");
}
