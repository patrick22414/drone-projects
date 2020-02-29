#include "Chase2D.h"

Chase2D::Chase2D(
    const std::string& connection, int camera, const eg::Vector2i& resolution, const std::string& video_output) :
    Chase2D(connection)
{
    capture = cv::VideoCapture(camera, cv::CAP_V4L);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, resolution[0]);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, resolution[1]);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open camera");

    if (!capture.read(im))
        log_red_and_exit("Cannot read frame");

    if (!video_output.empty()) {
        auto codec = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
        auto fps   = 30.0; // TODO

        writer = cv::VideoWriter(video_output, codec, fps, im.size());

        if (!writer.isOpened())
            log_red_and_exit("Cannot open video output to write");
    }

    this->resolution = resolution;
}

Chase2D::Chase2D(const std::string& connection, const std::string& video_input) : Chase2D(connection)
{
    capture = cv::VideoCapture(video_input);

    if (!capture.isOpened())
        log_red_and_exit("Cannot open video input");

    if (!capture.read(im))
        log_red_and_exit("Cannot read frame");

    this->resolution = {im.size().width, im.size().height};
}

// Private constructor to init MAVSDK connection
Chase2D::Chase2D(const std::string& connection)
{
    mav::Mavsdk mavsdk;

    auto connection_result = mavsdk.add_any_connection(connection);
    check_connection_result(connection_result);

    while (!mavsdk.is_connected()) {
        log_yellow("Waiting for vehicle to connect...");
        sleep_for(seconds(1));
    }

    mav::System& system = mavsdk.system();
    log_green("Vehicle connected");

    action    = std::make_shared<mav::Action>(system);
    offboard  = std::make_shared<mav::Offboard>(system);
    telemetry = std::make_shared<mav::Telemetry>(system);

    while (!telemetry->health_all_ok()) {
        log_yellow("Waiting for vehicle to be ready...");
        sleep_for(seconds(1));
    }

    log_green("Vehicle is ready to be armed");

    mav::Action::Result arm_result = action->arm();
    check_action_result(arm_result);
    log_green("Vehicle is Armed");

    log_green("Connection complete");
}
